//! UEFI `EFI_PCI_IO_PROTOCOL`-based DMA buffer allocation.
//!
//! On the Framework 13 AMD the firmware leaves the AMD-Vi IOMMU enabled in
//! pre-boot, so the HDA controller's DMA only reaches memory that the firmware
//! has mapped into the device's IOMMU domain. Raw allocations (Rust `alloc`,
//! UEFI `AllocatePages`) are NOT mapped, so the controller's DMA to them is
//! blocked — reads return zero (silence), writes vanish (zero RIRB/DPIB).
//!
//! `EFI_PCI_IO_PROTOCOL.AllocateBuffer` + `Map(BusMasterCommonBuffer)` asks the
//! firmware to allocate DMA-able memory and program the IOMMU mapping for the
//! device, returning the device-side (DMA) address to give to the controller.

#![allow(dead_code)]

use core::ffi::c_void;
use core::sync::atomic::{AtomicUsize, Ordering};

use r_efi::efi;
use r_efi::protocols::pci_io;

/// Pointer (as usize) to the matched EFI_PCI_IO_PROTOCOL for our controller.
static PCI_IO: AtomicUsize = AtomicUsize::new(0);

unsafe fn boot_services() -> *mut efi::BootServices {
    // Source of truth for the EFI System Table is `crate::env`. In std
    // mode that wraps `std::os::uefi::env::try_system_table()`; in no_std
    // mode it wraps the consumer-supplied pointer set via
    // `env::set_efi_globals(image_handle, system_table)`. Either way the
    // returned pointer is null if globals aren't initialised, so we
    // null-check before dereferencing.
    let st = crate::env::system_table() as *mut efi::SystemTable;
    if st.is_null() { return core::ptr::null_mut(); }
    (*st).boot_services
}

/// Locate the EFI_PCI_IO_PROTOCOL instance for the PCI device at
/// (target_bus, target_dev, target_func) and remember it for DMA allocation.
pub fn init(target_bus: usize, target_dev: usize, target_func: usize) -> bool {
    unsafe {
        let bs = boot_services();
        if bs.is_null() {
            crate::flog!("efidma: boot_services null");
            return false;
        }

        let mut guid = pci_io::PROTOCOL_GUID;
        let mut count: usize = 0;
        let mut handles: *mut efi::Handle = core::ptr::null_mut();
        let st = ((*bs).locate_handle_buffer)(
            efi::BY_PROTOCOL,
            &mut guid,
            core::ptr::null_mut(),
            &mut count,
            &mut handles,
        );
        if st.is_error() || handles.is_null() {
            crate::flog!("efidma: locate_handle_buffer failed st={:#X} count={}", st.as_usize(), count);
            return false;
        }
        crate::flog!("efidma: {} PciIo handle(s) found", count);

        for i in 0..count {
            let h = *handles.add(i);
            let mut pio: *mut c_void = core::ptr::null_mut();
            let mut g2 = pci_io::PROTOCOL_GUID;
            let st = ((*bs).handle_protocol)(h, &mut g2, &mut pio);
            if st.is_error() || pio.is_null() {
                continue;
            }
            let p = pio as *mut pci_io::Protocol;
            let (mut seg, mut bus, mut dev, mut func): (usize, usize, usize, usize) = (0, 0, 0, 0);
            let st = ((*p).get_location)(p, &mut seg, &mut bus, &mut dev, &mut func);
            if st.is_error() {
                continue;
            }
            if bus == target_bus && dev == target_dev && func == target_func {
                PCI_IO.store(p as usize, Ordering::SeqCst);
                crate::flog!("efidma: matched PciIo for {:02X}:{:02X}.{:X} (seg {})", bus, dev, func, seg);
                return true;
            }
        }
        crate::flog!("efidma: no PciIo matched {:02X}:{:02X}.{:X}", target_bus, target_dev, target_func);
        false
    }
}

pub fn is_available() -> bool {
    PCI_IO.load(Ordering::SeqCst) != 0
}

/// Allocate and intentionally leak `pages` pages of IOMMU-mapped DMA memory to
/// push the firmware's IOVA allocator forward, so subsequent (real) DMA buffers
/// land on fresh IOVAs.
///
/// Why: the firmware hands out the SAME device IOVAs every boot (e.g. 0x296000,
/// 0x297000, ...). After a power-off mid-DMA, AMD-Vi can retain stale IOTLB
/// entries for those reused IOVAs, so the controller's reads of the same IOVA
/// hit a stale/faulted translation (reads blocked, no error — our stuck-boot
/// signature). Consuming the previously-used IOVA range up front forces the
/// real buffers onto IOVAs the crashed boot never touched, dodging the stale
/// entries. The throwaway buffer is leaked (reclaimed when the app exits).
pub fn bump_iovas(pages: usize) {
    match alloc_common(pages) {
        Some((h, d)) => crate::flog!(
            "bump_iovas: leaked {} pages host={:#X} device={:#X} to shift real DMA onto fresh IOVAs",
            pages, h, d),
        None => crate::flog!("bump_iovas: alloc failed (no IOMMU? skipping)"),
    }
}

unsafe fn cfg_read(p: *mut pci_io::Protocol, width: pci_io::Width, offset: u32) -> u32 {
    let mut val: u32 = 0;
    let _ = ((*p).pci.read)(p, width, offset, 1, &mut val as *mut u32 as *mut c_void);
    val
}

unsafe fn cfg_write(p: *mut pci_io::Protocol, width: pci_io::Width, offset: u32, mut val: u32) {
    let _ = ((*p).pci.write)(p, width, offset, 1, &mut val as *mut u32 as *mut c_void);
}

/// Issue a PCIe Function Level Reset (FLR) to the controller, then restore the
/// 64-bit memory BAR0 and re-enable IO/MEM/bus-master in the command register.
///
/// Why: a power-off *while DMA is active* wedges the PCIe function's
/// bus-master/DMA state machine on the Framework 13 AMD. On the next (warm)
/// boot the HDA controller resets cleanly (GCTL CRST, stream SRST all succeed)
/// and the codec link is alive, but the controller's DMA engine reads NOTHING
/// from memory (SDLPIB and the DMA position buffer both stay 0, no error) —
/// only a full cold power-drain recovered it. GCTL CRST resets HDA-level logic
/// but NOT the PCIe function's DMA state machine; FLR does (this is what Linux
/// uses to recover wedged functions).
///
/// `bar0_phys` is the physical BAR0 base to restore (FLR clears BARs).
/// Returns true if an FLR was actually initiated.
pub fn function_level_reset(bar0_phys: u64) -> bool {
    unsafe {
        let p = PCI_IO.load(Ordering::SeqCst) as *mut pci_io::Protocol;
        if p.is_null() {
            crate::flog!("RESET: no PciIo");
            return false;
        }

        // Save command + a few config dwords FLR/PM-reset will clear, so we can
        // restore them and keep MMIO + DMA working afterwards.
        let cmd_before = cfg_read(p, pci_io::WIDTH_UINT16, 0x04) & 0xFFFF;

        // Walk the capability list once, recording the caps we can reset through:
        // PCIe (0x10), PCI Advanced Features (0x13), Power Management (0x01).
        let mut pcie_cap: u32 = 0;
        let mut af_cap: u32 = 0;
        let mut pm_cap: u32 = 0;
        let status = cfg_read(p, pci_io::WIDTH_UINT16, 0x06) & 0xFFFF;
        if status & (1 << 4) != 0 {
            let mut cap = (cfg_read(p, pci_io::WIDTH_UINT8, 0x34) & 0xFC) as u32;
            let mut guard = 0;
            while cap >= 0x40 && guard < 48 {
                match cfg_read(p, pci_io::WIDTH_UINT8, cap) & 0xFF {
                    0x10 => pcie_cap = cap,
                    0x13 => af_cap = cap,
                    0x01 => pm_cap = cap,
                    _ => {}
                }
                cap = cfg_read(p, pci_io::WIDTH_UINT8, cap + 1) & 0xFC;
                guard += 1;
            }
        }
        crate::flog!("RESET: caps pcie={:#X} af={:#X} pm={:#X} cmd_before={:#06X}",
            pcie_cap, af_cap, pm_cap, cmd_before);

        // Escalate exactly like Linux's pci_reset_function: PCIe FLR, then AF
        // FLR, then PM reset (D3hot->D0). Each does a real internal reset of the
        // PCIe function (incl. its DMA/bus-master state machine) which GCTL CRST
        // cannot touch — the state a power-off mid-DMA wedges.
        let mut method = "none";

        // 1. PCIe FLR — DEVCAP (pcie_cap+0x04) bit 28 = FLR capable.
        if pcie_cap != 0 {
            let devcap = cfg_read(p, pci_io::WIDTH_UINT32, pcie_cap + 0x04);
            if devcap & (1 << 28) != 0 {
                let devctl = cfg_read(p, pci_io::WIDTH_UINT16, pcie_cap + 0x08) & 0xFFFF;
                cfg_write(p, pci_io::WIDTH_UINT16, pcie_cap + 0x08, devctl | (1 << 15));
                crate::hda::delay_us(100_000); // up to 100ms per spec
                method = "pcie-flr";
            } else {
                crate::flog!("RESET: PCIe FLR not supported (devcap={:#010X})", devcap);
            }
        }

        // 2. AF FLR — AF Capabilities (af_cap+0x03) bit 1 = FLR capable;
        //    initiate via AF Control (af_cap+0x04) bit 0.
        if method == "none" && af_cap != 0 {
            let af_caps = cfg_read(p, pci_io::WIDTH_UINT8, af_cap + 0x03) & 0xFF;
            if af_caps & 0x02 != 0 {
                cfg_write(p, pci_io::WIDTH_UINT8, af_cap + 0x04, 0x01);
                crate::hda::delay_us(100_000);
                method = "af-flr";
            } else {
                crate::flog!("RESET: AF FLR not supported (af_caps={:#04X})", af_caps);
            }
        }

        // 3. PM reset — D3hot then D0. PMCSR = pm_cap+0x04; bit 3 = No_Soft_Reset
        //    (if set, a D3->D0 transition does NOT reset, so skip). Bits 0-1 =
        //    PowerState.
        if method == "none" && pm_cap != 0 {
            let pmcsr = cfg_read(p, pci_io::WIDTH_UINT16, pm_cap + 0x04) & 0xFFFF;
            if pmcsr & (1 << 3) == 0 {
                // -> D3hot
                cfg_write(p, pci_io::WIDTH_UINT16, pm_cap + 0x04, (pmcsr & !0x3) | 0x3);
                crate::hda::delay_us(10_000);
                // -> D0
                cfg_write(p, pci_io::WIDTH_UINT16, pm_cap + 0x04, pmcsr & !0x3);
                crate::hda::delay_us(10_000);
                method = "pm-reset";
            } else {
                crate::flog!("RESET: PM reset unavailable (No_Soft_Reset set, pmcsr={:#06X})", pmcsr);
            }
        }

        // Restore 64-bit BAR0 (0x10 low / 0x14 high) and command reg — a real
        // reset clears them.
        cfg_write(p, pci_io::WIDTH_UINT32, 0x10, (bar0_phys & 0xFFFF_FFF0) as u32);
        cfg_write(p, pci_io::WIDTH_UINT32, 0x14, (bar0_phys >> 32) as u32);
        cfg_write(p, pci_io::WIDTH_UINT16, 0x04, cmd_before | 0x7);

        let cmd_after = cfg_read(p, pci_io::WIDTH_UINT16, 0x04) & 0xFFFF;
        let bar0_after = cfg_read(p, pci_io::WIDTH_UINT32, 0x10);
        crate::flog!("RESET: method={} restored BAR0={:#010X}(readback={:#010X}) cmd={:#06X}",
            method, bar0_phys, bar0_after, cmd_after);
        method != "none"
    }
}

/// Allocate `pages` 4 KiB pages of cache-coherent DMA common-buffer memory.
/// Returns `(host_addr, device_addr)`: the host address is for CPU access, the
/// device address must be programmed into the controller's DMA registers.
/// Returns `None` if no PciIo is available (e.g. QEMU without IOMMU) so callers
/// can fall back to plain allocation.
pub fn alloc_common(pages: usize) -> Option<(usize, usize)> {
    unsafe {
        let p = PCI_IO.load(Ordering::SeqCst) as *mut pci_io::Protocol;
        if p.is_null() {
            return None;
        }

        let mut host: *mut c_void = core::ptr::null_mut();
        let st = ((*p).allocate_buffer)(
            p,
            efi::ALLOCATE_ANY_PAGES,
            efi::BOOT_SERVICES_DATA,
            pages,
            &mut host,
            0, // default attributes (coherent common buffer)
        );
        if st.is_error() || host.is_null() {
            crate::flog!("efidma: allocate_buffer({} pages) failed st={:#X}", pages, st.as_usize());
            return None;
        }
        core::ptr::write_bytes(host as *mut u8, 0, pages * 4096);

        let mut bytes: usize = pages * 4096;
        let mut device: efi::PhysicalAddress = 0;
        let mut mapping: *mut c_void = core::ptr::null_mut();
        let st = ((*p).map)(
            p,
            pci_io::OPERATION_BUS_MASTER_COMMON_BUFFER,
            host,
            &mut bytes,
            &mut device,
            &mut mapping,
        );
        if st.is_error() {
            crate::flog!("efidma: map failed st={:#X}", st.as_usize());
            return None;
        }
        crate::flog!("efidma: alloc {} pages host={:#X} device={:#X} mapped_bytes={}",
            pages, host as usize, device as usize, bytes);
        Some((host as usize, device as usize))
    }
}
