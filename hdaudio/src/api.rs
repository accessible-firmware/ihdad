//! High-level public API for embedding `ihdad` into other UEFI / BIOS code.
//!
//! Three entry points:
//!
//! 1. [`init_intel_hda`] — given a HDA controller's already-mapped BAR0 and
//!    PCI vendor/device id, bring the controller up, find the internal
//!    speaker (or the first available output pin), wire it to a DAC, and
//!    start the output stream. Returns a ready-to-write [`IntelHDA`].
//!
//! 2. [`speak_intel_hda`] — push a slice of interleaved PCM `i16` samples
//!    through the running output stream. Caller passes any sample rate /
//!    channel count and the function resamples + downmixes to the
//!    controller's native format ([`NATIVE_FORMAT`]) on the fly.
//!
//! 3. [`shutdown_intel_hda`] — stop all DMA, mask interrupts, and drive the
//!    controller into reset so the next consumer (e.g. Linux booted via
//!    GRUB) starts from cold.
//!
//! The intent is that a bootloader / firmware integrator can just call:
//!
//! ```ignore
//! let mut hda = unsafe { ihdad::init_intel_hda(bar0, vend_prod, Some((b,d,f)))? };
//! ihdad::speak_intel_hda(&mut hda, &samples, 22_050, 1);
//! ihdad::shutdown_intel_hda(&mut hda);
//! ```

use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::hda::{self, IntelHDA};
use crate::pcid::pci::{
    CfgAccess, Pci, PciBar, PciBus, PciHeader, PciHeaderError,
};
use crate::pcid::pcie::Pcie;

/// Audio format the controller is configured to consume on its single
/// output stream after [`init_intel_hda`]. Any PCM handed to
/// [`speak_intel_hda`] is converted to this format internally.
#[derive(Debug, Clone, Copy)]
pub struct AudioFormat {
    pub sample_rate_hz: u32,
    pub channels: u8,
    pub bits_per_sample: u8,
}

/// The fixed format `init_intel_hda` programs the output stream for.
///
/// 48 kHz / 16-bit signed / stereo is the only format guaranteed to be
/// supported by every HDA codec per the HDA 1.0a spec, so the library
/// uses it unconditionally and converts caller PCM to match.
pub const NATIVE_FORMAT: AudioFormat = AudioFormat {
    sample_rate_hz: 48_000,
    channels: 2,
    bits_per_sample: 16,
};

/// Sample format the controller's output stream is configured for.
/// Library callers can use this to size their resampling target.
pub fn native_format() -> AudioFormat {
    NATIVE_FORMAT
}

/// Set up the HDA controller for output.
///
/// `bar0_phys` is the BAR0 base address (must already be mapped and the
/// PCI command register's Memory Space + Bus Master bits enabled — the
/// PCI enumeration in `main.rs` is one reference for that). `vend_prod`
/// is `(vendor << 16) | device`, used to pick controller workarounds.
///
/// If `pci_io_info = Some((bus, dev, func))`, the `EFI_PCI_IO_PROTOCOL`
/// instance for that location is retrieved and used to allocate
/// IOMMU-mapped DMA buffers. This is required on real AMD / Intel
/// laptop hardware where the firmware leaves the IOMMU enabled in
/// pre-boot. Pass `None` in IOMMU-less environments (plain QEMU, etc.) —
/// allocations then fall back to the Rust global allocator.
///
/// On return the device has:
///   - codecs enumerated, widget tree walked;
///   - the internal speaker pin (or the first output pin if no speaker
///     is present) bound to a DAC and unmuted;
///   - output stream 0 configured for [`NATIVE_FORMAT`] and running.
///
/// # Safety
/// Caller must guarantee that `bar0_phys` points at the HDA controller's
/// MMIO region exclusively — the library writes registers under that
/// address.
pub unsafe fn init_intel_hda(
    bar0_phys: usize,
    vend_prod: u32,
    pci_io_info: Option<(usize, usize, usize)>,
) -> crate::io_compat::Result<IntelHDA> {
    if let Some((b, d, f)) = pci_io_info {
        if crate::efidma::init(b, d, f) {
            let _ = crate::efidma::function_level_reset(bar0_phys as u64);
            crate::efidma::bump_iovas(48);
        }
    }

    let mut device = unsafe { IntelHDA::new(bar0_phys, vend_prod)? };

    let pin_count = device.output_pin_count();
    if pin_count == 0 {
        return Err(crate::io_compat::Error::new(
            crate::io_compat::ErrorKind::NotFound,
            "ihdad: HDA controller has no output pins",
        ));
    }

    // Prefer the internal speaker — that's the one a blind user needs
    // for TTS during firmware phase. Fall back to the first output pin.
    let mut chosen = 0usize;
    for i in 0..pin_count {
        if device.output_pin_info_str(i).contains("Speaker") {
            chosen = i;
            break;
        }
    }
    let pin_addr = device.output_pin_addr(chosen);

    if device.configure_pin_for_output(pin_addr).is_none() {
        return Err(crate::io_compat::Error::new(
            crate::io_compat::ErrorKind::Other,
            "ihdad: no DAC reachable from chosen output pin",
        ));
    }

    device.setup_output_stream();
    Ok(device)
}

/// A single HDA controller found by [`discover_intel_hda`]. Has already
/// had bus mastering enabled and PCIe snoop / no-snoop policies fixed up
/// — i.e. it's ready to hand to [`init_intel_hda`].
#[derive(Debug, Clone, Copy)]
pub struct HdaController {
    pub bus: u8,
    pub dev: u8,
    pub func: u8,
    pub vendor_id: u16,
    pub device_id: u16,
    /// Physical / bus address of BAR0 (the HDA register window).
    pub bar0_phys: u64,
    /// Size of BAR0 in bytes (from the standard BAR-sizing dance).
    pub bar0_size: u32,
}

impl HdaController {
    /// `(vendor << 16) | device` — the form `IntelHDA::new` wants.
    pub fn vend_prod(&self) -> u32 {
        ((self.vendor_id as u32) << 16) | (self.device_id as u32)
    }
}

/// Walk the PCI bus tree and return every HD Audio controller
/// (class = 0x04 Multimedia, subclass = 0x03 HDA) present.
///
/// For each match this also:
///   - sets Bus Master + Memory Space + I/O Space in the command register;
///   - applies the vendor-specific PCIe snoop fix-up
///     (Intel TCSEL = TC0, AMD snoop enable, Nvidia snoop enable) so that
///     DMA is cache-coherent;
///   - clears the PCIe `No Snoop Enable` bit on the device-control register
///     so the controller's DMA must be snooped (a redundant safety belt for
///     systems where the vendor fix-up alone isn't enough);
///   - assigns a legacy IRQ line of 9 if firmware left it at 0xFF;
///   - measures BAR0's size via the standard write-all-ones / read-back.
///
/// Bridges are walked breadth-first so devices behind PCI-to-PCI bridges
/// (most laptop / desktop chipsets nowadays) are discovered.
pub fn discover_intel_hda() -> Vec<HdaController> {
    let pci = Arc::new(Pci::new());
    let pcie = Pcie::new(Arc::clone(&pci)).ok();
    let cfg: &dyn CfgAccess = match pcie.as_ref() {
        Some(p) => p,
        None => &*pci,
    };

    let mut found: Vec<HdaController> = Vec::new();

    // BFS so PCI-to-PCI bridges have their secondary bus added to the queue.
    let mut bus_queue: Vec<u8> = vec![0];
    let mut bus_i = 0;
    while bus_i < bus_queue.len() {
        let bus_num = bus_queue[bus_i];
        bus_i += 1;

        let bus = PciBus { pci: cfg, num: bus_num };
        for dev in bus.devs() {
            for f in dev.funcs() {
                let func_num = f.num;
                let header = match PciHeader::from_reader(f) {
                    Ok(h) => h,
                    Err(PciHeaderError::NoDevice) => continue,
                    Err(_) => continue,
                };

                if let PciHeader::PciToPci { secondary_bus_num, .. } = header {
                    bus_queue.push(secondary_bus_num);
                }

                let class_raw: u8 = header.class().into();
                if class_raw != 0x04 || header.subclass() != 0x03 {
                    continue;
                }

                let ctrl = configure_hda_function(cfg, bus_num, dev.num, func_num, &header);
                found.push(ctrl);
            }
        }
    }

    found
}

/// Apply the same per-function programming as `main.rs`'s
/// `handle_parsed_header` does for a matched HDA controller, and return
/// a summary suitable for [`init_intel_hda`].
fn configure_hda_function(
    cfg: &dyn CfgAccess,
    bus_num: u8,
    dev_num: u8,
    func_num: u8,
    header: &PciHeader,
) -> HdaController {
    unsafe {
        // Enable Bus Master | Memory Space | I/O Space.
        let mut cmd = cfg.read(bus_num, dev_num, func_num, 0x04);
        cmd |= 0x7;
        cfg.write(bus_num, dev_num, func_num, 0x04, cmd);

        // Vendor-specific PCIe snoop / traffic-class fix-up. Without this,
        // DMA is not cache-coherent on several controllers.
        match header.vendor_id() {
            // Intel: clear TCSEL[2:0] so traffic class 0 is selected.
            0x8086 => {
                let tcsel = cfg.read(bus_num, dev_num, func_num, 0x44);
                cfg.write(bus_num, dev_num, func_num, 0x44, tcsel & 0xFFFF_FFF8);
            }
            // AMD: enable PCIe snoop at config byte 0x42 (bit-level fixup).
            0x1022 | 0x1002 => {
                let reg40 = cfg.read(bus_num, dev_num, func_num, 0x40);
                let byte42 = (reg40 >> 16) & 0xFF;
                let new_byte42 = (byte42 & 0xF8) | 0x02;
                let new_reg40 = (reg40 & 0xFF00_FFFF) | (new_byte42 << 16);
                cfg.write(bus_num, dev_num, func_num, 0x40, new_reg40);
            }
            // Nvidia: enable snoop at config byte 0x4E.
            0x10DE => {
                let reg4c = cfg.read(bus_num, dev_num, func_num, 0x4C);
                let byte4e = (reg4c >> 16) & 0xFF;
                let new_byte4e = (byte4e & 0xF0) | 0x0F;
                let new_reg4c = (reg4c & 0xFF00_FFFF) | (new_byte4e << 16);
                cfg.write(bus_num, dev_num, func_num, 0x4C, new_reg4c);
            }
            _ => {}
        }

        // Walk the capability list and clear PCIe `No Snoop Enable` (DevCtl
        // bit 11) so the controller's DMA must be snooped. This is the
        // redundant belt to the vendor fixup's braces.
        let status = (cfg.read(bus_num, dev_num, func_num, 0x04) >> 16) as u16;
        if status & (1 << 4) != 0 {
            let mut cap_ptr = (cfg.read(bus_num, dev_num, func_num, 0x34) & 0xFC) as u8;
            let mut guard = 0;
            while cap_ptr >= 0x40 && guard < 48 {
                let capw = cfg.read(bus_num, dev_num, func_num, cap_ptr as u16);
                let cap_id = (capw & 0xFF) as u8;
                let next = ((capw >> 8) & 0xFF) as u8;
                if cap_id == 0x10 {
                    let devctl_off = cap_ptr as u16 + 0x08;
                    let dc = cfg.read(bus_num, dev_num, func_num, devctl_off);
                    let devctl = (dc & 0xFFFF) as u16;
                    let new_devctl = devctl & !(1 << 11);
                    cfg.write(
                        bus_num, dev_num, func_num, devctl_off,
                        (dc & 0xFFFF_0000) | new_devctl as u32,
                    );
                }
                cap_ptr = next;
                guard += 1;
            }
        }

        // Assign an IRQ line of 9 if firmware left it at 0xFF.
        let mut irq_reg = cfg.read(bus_num, dev_num, func_num, 0x3C);
        if (irq_reg & 0xFF) as u8 == 0xFF {
            irq_reg = (irq_reg & 0xFFFF_FF00) | 9;
            cfg.write(bus_num, dev_num, func_num, 0x3C, irq_reg);
        }

        // BAR0 + its size (standard write-1s / read-back dance).
        let bar0 = header.get_bar(0);
        let original = cfg.read(bus_num, dev_num, func_num, 0x10);
        cfg.write(bus_num, dev_num, func_num, 0x10, 0xFFFF_FFFF);
        let new = cfg.read(bus_num, dev_num, func_num, 0x10);
        cfg.write(bus_num, dev_num, func_num, 0x10, original);
        let masked = if new & 1 == 1 {
            new & 0xFFFF_FFFC
        } else {
            new & 0xFFFF_FFF0
        };
        let bar0_size = (!masked).wrapping_add(1);
        let bar0_phys = match bar0 {
            PciBar::Memory32(p) => p as u64,
            PciBar::Memory64(p) => p,
            _ => 0,
        };

        HdaController {
            bus: bus_num,
            dev: dev_num,
            func: func_num,
            vendor_id: header.vendor_id(),
            device_id: header.device_id(),
            bar0_phys,
            bar0_size: if bar0_size <= 1 { 0 } else { bar0_size },
        }
    }
}

/// One-call setup: discover HDA controllers, pick the best one (chipset
/// audio over GPU HDMI audio — GPU HDMI codecs only respond when a display
/// link is active, so they're a poor default for boot-time TTS), and run
/// [`init_intel_hda`] on it.
///
/// Intended as the integration point for downstream firmware code (GRUB,
/// custom bootloaders) that don't want to drive PCI enumeration themselves.
///
/// # Safety
/// Writes to PCI configuration space and to the controller's MMIO BAR.
/// Caller must guarantee that no other code (kernel, firmware driver) is
/// currently driving this device.
pub unsafe fn init_intel_hda_auto() -> crate::io_compat::Result<IntelHDA> {
    let mut controllers = discover_intel_hda();
    if controllers.is_empty() {
        return Err(crate::io_compat::Error::new(
            crate::io_compat::ErrorKind::NotFound,
            "ihdad: no HD Audio controller found on the PCI bus",
        ));
    }

    // Prefer chipset audio (Intel, AMD chipset, etc.) over GPU HDMI audio
    // (AMD/ATI vendor 0x1002, Nvidia 0x10DE which is GPU-side audio when
    // it's on a discrete GPU). GPU HDMI codecs only respond if a monitor
    // link is up, which is a coin flip during firmware phase.
    controllers.sort_by_key(|c| match c.vendor_id {
        0x1002 => 2u8,
        0x10DE => 2u8,
        _ => 0u8,
    });

    let chosen = controllers[0];
    if chosen.bar0_phys == 0 {
        return Err(crate::io_compat::Error::new(
            crate::io_compat::ErrorKind::InvalidData,
            "ihdad: chosen HDA controller has BAR0 mapped to address 0",
        ));
    }

    unsafe {
        init_intel_hda(
            chosen.bar0_phys as usize,
            chosen.vend_prod(),
            Some((chosen.bus as usize, chosen.dev as usize, chosen.func as usize)),
        )
    }
}

/// Cleanly tear the HDA controller down so a subsequent OS (Linux booted
/// via GRUB, etc.) finds it in cold-reset state.
///
/// Equivalent to [`IntelHDA::shutdown`]; kept as a free function so the
/// shutdown / speak / init triplet sits at the same call site style.
pub fn shutdown_intel_hda(device: &mut IntelHDA) {
    device.shutdown();
}

// Each cyclic-ring sub-buffer is 2 KiB = 512 frames of stereo i16 at
// 48 kHz, matching `IntelHDA::setup_output_stream`.
const BLOCK_SIZE: usize = 2048;

// Brief silence written at the head of every utterance so the DAC has
// settled before the first phoneme arrives — without this a short
// utterance can have its opening clipped.
const LEAD_IN_FRAMES: usize = 1200;

// After an utterance ends, overwrite the WHOLE ring (32 blocks) plus
// margin with silence so the DMA can't loop back over the tail of the
// just-finished clip while we wait for the next call.
const TAIL_BLOCKS: usize = 40;

/// Push interleaved PCM `i16` samples through the controller's output
/// stream. `sample_rate_hz` and `channels` describe the caller's data;
/// the function resamples (linear interpolation) to 48 kHz and downmixes
/// (channel-average) or duplicates to stereo as needed.
///
/// Blocks the caller while the audio is queued and the ring is flushed
/// with silence. Returns early without panicking if the DMA stalls.
pub fn speak_intel_hda(
    device: &mut IntelHDA,
    samples: &[i16],
    sample_rate_hz: u32,
    channels: u8,
) {
    if samples.is_empty() || channels == 0 || sample_rate_hz == 0 {
        return;
    }

    let in_channels = channels as usize;
    let in_rate = sample_rate_hz as u64;
    let out_rate = NATIVE_FORMAT.sample_rate_hz as u64;
    let in_frames = samples.len() / in_channels;
    let out_frames = (in_frames as u64 * out_rate / in_rate) as usize;

    // Start writing just ahead of the DMA read cursor so the clip plays
    // in order from its first sample — critical for short utterances.
    device.align_write_cursor(2);

    let mut block = [0u8; BLOCK_SIZE];
    let mut bi: usize = 0;

    // Lead-in silence.
    for _ in 0..LEAD_IN_FRAMES {
        block[bi] = 0;
        block[bi + 1] = 0;
        block[bi + 2] = 0;
        block[bi + 3] = 0;
        bi += 4;
        if bi == BLOCK_SIZE {
            if !write_block_blocking(device, &block) {
                return;
            }
            bi = 0;
        }
    }

    let frame_mono = |f: usize| -> i32 {
        if f >= in_frames {
            return 0;
        }
        let base = f * in_channels;
        if in_channels == 1 {
            samples[base] as i32
        } else {
            let mut s: i32 = 0;
            for c in 0..in_channels {
                s += samples[base + c] as i32;
            }
            s / in_channels as i32
        }
    };

    let in_rate_f = in_rate as f64;
    let out_rate_f = out_rate as f64;
    for o in 0..out_frames {
        let pos = o as f64 * in_rate_f / out_rate_f;
        let i0 = pos as usize;
        let frac = pos - i0 as f64;
        let s0 = frame_mono(i0);
        let s1 = frame_mono(i0 + 1);
        let interp = s0 as f64 + (s1 - s0) as f64 * frac;
        // Attenuate ~50% — laptop internal speakers are loud at full scale.
        let s = (interp as i32 / 2).clamp(i16::MIN as i32, i16::MAX as i32) as i16;
        let b = s.to_le_bytes();
        block[bi]     = b[0];
        block[bi + 1] = b[1];
        block[bi + 2] = b[0]; // duplicate L -> R
        block[bi + 3] = b[1];
        bi += 4;
        if bi == BLOCK_SIZE {
            if !write_block_blocking(device, &block) {
                return;
            }
            bi = 0;
        }
    }

    if bi > 0 {
        for k in bi..BLOCK_SIZE {
            block[k] = 0;
        }
        let _ = write_block_blocking(device, &block);
    }

    let silence = [0u8; BLOCK_SIZE];
    for _ in 0..TAIL_BLOCKS {
        if !write_block_blocking(device, &silence) {
            break;
        }
    }
    hda::delay_us(150_000);
}

// Write one 2 KiB block of PCM to output stream 0, retrying until the
// ring has drained enough to accept it. Bails after ~1.2 s of stall.
fn write_block_blocking(device: &mut IntelHDA, block: &[u8]) -> bool {
    let mut retries = 0u32;
    loop {
        match device.write_to_output(0, block) {
            Ok(Some(_)) => return true,
            Ok(None) => {
                retries += 1;
                if retries > 12_000 {
                    return false;
                }
                hda::delay_us(100);
            }
            Err(_) => return false,
        }
    }
}
