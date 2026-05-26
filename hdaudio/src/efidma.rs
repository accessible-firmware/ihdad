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
    let st = std::os::uefi::env::system_table().as_ptr() as *mut efi::SystemTable;
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
