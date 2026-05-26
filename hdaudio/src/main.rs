#![allow(dead_code)]
#![feature(uefi_std)]

mod hda;
mod pcid;
mod efidma;


use pcid::*;
use crate::state::*;





use pcid::pci::cap::CapabilityOffsetsIter;
use pcid::pci::cap::CapabilitiesIter;
use pcid::SubdriverArguments;






use pcid::PcidServerHandle;
use crate::irq_helpers::{read_bsp_apic_id, allocate_single_interrupt_vector};

#[cfg(not(target_arch = "x86_64"))]
fn get_int_method(pcid_handle: &mut PcidServerHandle) -> Option<File> {
    let pci_config = pcid_handle.fetch_config().expect("ihdad: failed to fetch config");
    let irq = pci_config.func.legacy_interrupt_line;

    if pci_config.func.legacy_interrupt_pin.is_some() {
        // legacy INTx# interrupt pins.
        Some(File::open(format!("irq:{}", irq)).expect("ihdad: failed to open legacy IRQ file"))
    } else {
        // no interrupts at all
        None
    }
}

#[cfg(target_arch = "x86_64")]
fn get_int_method(pcid_handle: &mut PcidServerHandle) -> Option<File> {
    let pci_config = pcid_handle.fetch_config().expect("ihdad: failed to fetch config");

    let irq = pci_config.func.legacy_interrupt_line;

    let all_pci_features = pcid_handle.fetch_all_features().expect("ihdad: failed to fetch pci features");
    std::println!("PCI FEATURES: {:?}", all_pci_features);

    let (has_msi, mut msi_enabled) = all_pci_features.iter().map(|(feature, status)| (feature.is_msi(), status.is_enabled())).find(|&(f, _)| f).unwrap_or((false, false));
    let (has_msix, mut msix_enabled) = all_pci_features.iter().map(|(feature, status)| (feature.is_msix(), status.is_enabled())).find(|&(f, _)| f).unwrap_or((false, false));

    if has_msi && !msi_enabled && !has_msix {
        msi_enabled = true;
    }
    if has_msix && !msix_enabled {
        msix_enabled = true;
    }

    if msi_enabled && !msix_enabled {
        use pcid::msi::x86_64::{DeliveryMode, self as x86_64_msix};

        let _capability = match pcid_handle.feature_info(PciFeature::Msi).expect("ihdad: failed to retrieve the MSI capability structure from pcid") {
            PciFeatureInfo::Msi(s) => s,
            PciFeatureInfo::MsiX(_) => std::panic!(),
        };
        // TODO: Allow allocation of up to 32 vectors.

        // TODO: Find a way to abstract this away, potantially as a helper module for
        // pcid_interface, so that this can be shared between nvmed, xhcid, ixgebd, etc..

        let destination_id = read_bsp_apic_id().expect("ihdad: failed to read BSP apic id");
        let lapic_id = u8::try_from(destination_id).expect("CPU id didn't fit inside u8");
        let msg_addr = x86_64_msix::message_address(lapic_id, false, false);

        let (vector, interrupt_handle) = allocate_single_interrupt_vector(destination_id).expect("ihdad: failed to allocate interrupt vector").expect("ihdad: no interrupt vectors left");
        let msg_data = x86_64_msix::message_data_edge_triggered(DeliveryMode::Fixed, vector);

        let set_feature_info = MsiSetFeatureInfo {
            multi_message_enable: Some(0),
            message_address: Some(msg_addr),
            message_upper_address: Some(0),
            message_data: Some(msg_data as u16),
            mask_bits: None,
        };
        pcid_handle.set_feature_info(SetFeatureInfo::Msi(set_feature_info)).expect("ihdad: failed to set feature info");

        pcid_handle.enable_feature(PciFeature::Msi).expect("ihdad: failed to enable MSI");
        std::println!("Enabled MSI");

        Some(interrupt_handle)
    } else if pci_config.func.legacy_interrupt_pin.is_some() {
        std::println!("Legacy IRQ {}", irq);

        // legacy INTx# interrupt pins.
        Some(File::open(format!("irq:{}", irq)).expect("ihdad: failed to open legacy IRQ file"))
    } else {
        // no interrupts at all
        None
    }
}

fn main() {
  // Initialize file logging early so PCI-scan-phase diagnostics are captured.
  hda::filelog::init();
  println!("Attempt to create PCI channels: ");
  let (mut pci_from_write, pci_from_read) = bounded(1024);
  println!("OK");
  let (pci_to_write, mut pci_to_read) = bounded(1024);
  println!("OK2");
  let drivers = pci_main(&mut pci_from_write, &mut pci_to_read);
  hda_main(pci_to_write, pci_from_read, pci_from_write, pci_to_read, drivers);
  println!("Sucessfully initialized Intel HD Audio buffers. TBC!");
}

fn hda_main(pci_to_write: Sender<Vec<u8>>, pci_from_read: Receiver<Vec<u8>>, pci_from_write: Sender<Vec<u8>>, pci_to_read: Receiver<Vec<u8>>, drivers: Vec<(DriverHandler, SubdriverArguments)>) {
  std::println!("Opening Intel HDA");

  // Print all found HDA controllers for diagnostics
  println!("Found {} HDA controller(s):", drivers.len());
  for (i, (_, args)) in drivers.iter().enumerate() {
    println!("  [{}] {:04X}:{:04X} bus={:02X} dev={:02X} func={:02X} BAR0={}",
      i, args.func.venid, args.func.devid,
      args.func.bus_num, args.func.dev_num, args.func.func_num,
      args.func.bars[0]);
  }

  // Sort: prefer chipset HDA controllers (vendor 0x1022, 0x8086, etc.)
  // over GPU HDMI/DP audio controllers (vendor 0x1002 = AMD/ATI GPU).
  // GPU HDMI audio codecs won't respond unless a display link is active.
  let mut drivers = drivers;
  drivers.sort_by_key(|(_, args)| match args.func.venid {
    0x1002 => 1u8, // AMD/ATI GPU HDMI audio — deprioritize
    _ => 0u8,      // Chipset HDA controllers — prefer
  });

  println!("Selected controller: {:04X}:{:04X}",
    drivers[0].1.func.venid, drivers[0].1.func.devid);

  let mut pcid_handle = PcidServerHandle::connect(pci_to_write, pci_to_read, pci_from_write, pci_from_read, drivers).expect("ihdad: failed to setup channel to pcid");
  std::println!("Setup PCID Handle"); 

  let pci_config = pcid_handle.fetch_config().expect("ihdad: failed to fetch config");
  std::println!("Setup PCI Config"); 

  let mut name = pci_config.func.name();
  name.push_str("_ihda");
  std::println!("Name added _ihda"); 

  let bar = pci_config.func.bars[0];
  let bar_size = pci_config.func.bar_sizes[0];
  let bar_ptr = match bar {
      pcid::PciBar::Memory32(ptr) => match ptr {
          0 => std::panic!("BAR 0 is mapped to address 0"),
          _ => ptr as u64,
      },
      pcid::PciBar::Memory64(ptr) => match ptr {
          0 => std::panic!("BAR 0 is mapped to address 0"),
          _ => ptr,
      },
      other => std::panic!("Expected memory bar, found {}", other),
  };
  std::println!("Set up bar!"); 

  std::println!(" + IHDA {} on: {:#X} size: {}", name, bar_ptr, bar_size);

   let address = match bar_ptr.try_into() {
      Ok(addr) => addr,
      Err(_) => {
        println!("ERROR CONVERTING ADDRESS POINTER! FATAL!");
        panic!();
      },
   };
   println!("Converted address pointer!");
/*
let address = unsafe {
	common::physmap(bar_ptr as usize, bar_size as usize, common::Prot::RW, common::MemoryType::Uncacheable)
		.expect("ihdad: failed to map address") as usize
};
*/

  //TODO: MSI-X
  //let mut irq_file = get_int_method(&mut pcid_handle).expect("ihdad: no interrupt file");
  //println!("Made IRQ file");

	let vend_prod:u32 = ((pci_config.func.venid as u32) << 16) | (pci_config.func.devid as u32);
  println!("Makde vendor prod");

  // Locate the firmware's EFI_PCI_IO_PROTOCOL for this controller so DMA
  // buffers can be allocated/mapped through the IOMMU (pre-boot DMA works).
  let dma_ok = efidma::init(
    pci_config.func.bus_num as usize,
    pci_config.func.dev_num as usize,
    pci_config.func.func_num as usize,
  );
  println!("PciIo DMA mapping available: {}", dma_ok);

	let mut device = unsafe { hda::IntelHDA::new(address, vend_prod).expect("ihdad: failed to allocate device") };
  println!("Created intel HDA device; starting pin enumeration test...");

  enumerate_and_test_outputs(&mut device);

  println!("Output pin enumeration complete.");
}

/// Enumerate all output pins and play a short tone on each one.
/// This allows the user to identify which pin produces audible output
/// by correlating the on-screen pin index with what they hear.
fn enumerate_and_test_outputs(device: &mut hda::IntelHDA) {
    let pin_count = device.output_pin_count();

    println!("==============================================");
    println!("  OUTPUT PIN ENUMERATION TEST");
    println!("  {} output pin(s) found", pin_count);
    println!("==============================================");

    // Print all pins first
    for i in 0..pin_count {
        println!("  [{}] {}", i, device.output_pin_info_str(i));
    }
    println!("----------------------------------------------");

    if pin_count == 0 {
        println!("ERROR: No output pins found! Cannot test.");
        return;
    }

    // Target the INTERNAL SPEAKER (the device the blind user needs for TTS).
    let mut speaker_idx = 0usize;
    let mut found_speaker = false;
    for i in 0..pin_count {
        if device.output_pin_info_str(i).contains("Speaker") {
            speaker_idx = i;
            found_speaker = true;
            break;
        }
    }
    let speaker = device.output_pin_addr(speaker_idx);
    println!("");
    println!("==============================================");
    println!("  STATIC TONE TEST (speaker found={})", found_speaker);
    println!("  Speaker pin: {}", device.output_pin_info_str(speaker_idx));
    println!("==============================================");

    match device.configure_pin_for_output(speaker) {
        Some(dac) => {
            println!("  speaker configured via DAC {:02X}:{:02X}", dac.0, dac.1);

            // Set up + START the output stream AFTER the DAC converter is bound
            // to it. Starting RUN before a converter consumes the stream lets
            // the FIFO fill and the DMA stall (POS stuck at 0) — a race.
            device.setup_output_stream();

            // Greeting so it's audibly working.
            println!("  SPEAKING via fry: \"hello uefi\"");
            speak_fry(device, "hello uefi");

            // Interactive prompt: speak whatever the user types.
            loop {
                println!("");
                println!("Type text to speak (letters a-z and spaces, max 32; empty line = quit):");
                let line = read_line_console();
                let cleaned: String = line
                    .trim()
                    .to_ascii_lowercase()
                    .chars()
                    .filter(|c| (*c >= 'a' && *c <= 'z') || *c == ' ')
                    .take(32)
                    .collect();
                if cleaned.trim().is_empty() {
                    println!("(done)");
                    break;
                }
                println!("Speaking: \"{}\"", cleaned);
                speak_fry(device, &cleaned);
            }

            // Stop the stream so the controller doesn't play freed memory
            // (garbage noise) after we return to the UEFI shell.
            device.stop_all_output();
        }
        None => {
            println!("  ERROR: no path to DAC from speaker pin {:02X}:{:02X}", speaker.0, speaker.1);
        }
    }

    println!("");
    println!("==============================================");
    println!("  STATIC TONE TEST COMPLETE");
    println!("==============================================");
}

/// Read a line from the UEFI console input protocol (ConIn). The UEFI std
/// target does not wire up std::io::stdin to the keyboard, so we poll
/// ReadKeyStroke directly. Echoes typed characters; returns the line without
/// the trailing newline (empty string if Enter is pressed with no input).
fn read_line_console() -> String {
    use r_efi::efi;
    use r_efi::protocols::simple_text_input as sti;
    use std::io::Write;

    let mut s = String::new();
    unsafe {
        let st = std::os::uefi::env::system_table().as_ptr() as *mut efi::SystemTable;
        if st.is_null() { return s; }
        let con_in = (*st).con_in;
        if con_in.is_null() { return s; }
        loop {
            let mut key = sti::InputKey::default();
            let status = ((*con_in).read_key_stroke)(con_in, &mut key);
            if status.is_error() {
                // EFI_NOT_READY: no key pending — poll again shortly.
                crate::hda::delay_us(2000);
                continue;
            }
            let ch = key.unicode_char;
            if ch == 0x000D {            // Enter (carriage return)
                println!("");
                break;
            } else if ch == 0x0008 {     // Backspace
                if s.pop().is_some() {
                    print!("\u{8} \u{8}");
                    let _ = std::io::stdout().flush();
                }
            } else if (0x20..0x7F).contains(&ch) {
                let c = ch as u8 as char;
                s.push(c);
                print!("{}", c);         // echo
                let _ = std::io::stdout().flush();
            }
        }
    }
    s
}

/// Write one 2048-byte audio block to output stream 0, retrying while the ring
/// is full (paced against the DMA read position).
/// Returns false if the write stalled (DMA not draining the ring) so the
/// caller can abort the utterance fast instead of hanging.
fn write_block_blocking(device: &mut hda::IntelHDA, block: &[u8]) -> bool {
    let mut retries = 0u32;
    loop {
        match device.write_to_output(0, block) {
            Ok(Some(_)) => return true,
            Ok(None) => {
                retries += 1;
                if retries == 8_000 {
                    let (ctl, sts, pos) = device.output_stream_diag(0);
                    crate::flog!("write STALL: retries={} SD_CTL={:#X} SD_STS={:#X} POS={:#X}", retries, ctl, sts, pos);
                    crate::hda::filelog::flush();
                }
                if retries > 12_000 {
                    crate::flog!("write_block_blocking: stalled, aborting utterance");
                    crate::hda::filelog::flush();
                    return false;
                }
                hda::delay_us(100);
            }
            Err(_) => return false,
        }
    }
}

/// Synthesize `text` with the `fry` TTS crate and stream it to the speaker.
///
/// `fry` outputs MONO i16 at 22050 Hz and only has lowercase a-z + space
/// samples, so the text is lowercased. We resample on the fly to the stream's
/// 48000 Hz stereo format and attenuate to a comfortable volume.
fn speak_fry(device: &mut hda::IntelHDA, text: &str) {
    const FRY_STRIDE: usize = 20810;   // samples per letter slot in fry's buffer
    const FRY_BUF_LEN: usize = 665920; // fry::MAX_BUFFER_SIZE (i16 samples)
    const FRY_RATE: u64 = 22050;
    const OUT_RATE: u64 = 48000;
    const BLOCK: usize = 2048;         // bytes; 512 stereo frames

    let mut buf: Vec<i16> = vec![0i16; FRY_BUF_LEN];
    let arr: &mut [i16; FRY_BUF_LEN] = match (&mut buf[..]).try_into() {
        Ok(a) => a,
        Err(_) => { println!("fry: buffer size mismatch"); return; }
    };

    let lowered = text.to_ascii_lowercase();
    let letters = match fry::tts(&lowered, arr) {
        Some(n) => n,
        None => { println!("fry: tts() failed (too long or unsupported char)"); return; }
    };
    // Each fry letter slot is 20810 samples but only part is real audio (the
    // rest is silence), and letters vary in length ('e'~2.7k vs 'w'~7.3k). Trim
    // each letter DYNAMICALLY to its actual audio extent + a short decay tail,
    // then a small inter-letter gap, so spelling is snappy without cutting long
    // letters. A space becomes a brief word-separator pause.
    const LETTER_DATA: usize = 10405; // samples of audio per letter in fry's data
    const TAIL: usize = 700;          // decay samples kept after the audio ends
    const THRESH: i32 = 200;          // amplitude considered "audio"
    const MAX_LEN: usize = 9000;      // safety cap per letter
    const LETTER_GAP: usize = 1100;   // short silence between letters (~0.05 s)
    const LEAD_IN: usize = 1200;      // brief silence lead-in (margin against clipping the start)

    let mut mono: Vec<i16> = Vec::with_capacity(LEAD_IN + letters * 6000);
    for _ in 0..LEAD_IN {
        mono.push(0);
    }
    for (l, ch) in lowered.chars().enumerate() {
        let start = l * FRY_STRIDE;
        if ch == ' ' {
            // Word separator: a slightly longer pause, no audio.
            for _ in 0..(LETTER_GAP * 3) { mono.push(0); }
            continue;
        }
        // Find this letter's real audio extent (last sample above threshold).
        let end = (start + LETTER_DATA).min(buf.len());
        let mut ext = 0usize;
        for i in start..end {
            if (buf[i] as i32).abs() > THRESH {
                ext = i - start + 1;
            }
        }
        let audio_len = (ext + TAIL).min(LETTER_DATA).min(MAX_LEN);
        for k in 0..audio_len {
            mono.push(*buf.get(start + k).unwrap_or(&0));
        }
        for _ in 0..LETTER_GAP {
            mono.push(0);
        }
    }
    println!("  fry: {} letters -> {} mono samples (dynamic trim)", letters, mono.len());

    // Start writing just ahead of the DMA read pointer so the clip plays in
    // order from the first sample (critical for short clips).
    device.align_write_cursor(2);
    crate::flog!("speak_fry: aligned, streaming {} mono samples", mono.len());
    crate::hda::filelog::flush();

    let out_frames = (mono.len() as u64 * OUT_RATE / FRY_RATE) as usize;
    let mut block = [0u8; BLOCK];
    let mut bi = 0usize;

    for o in 0..out_frames {
        // Linear-interpolated resample 22050 -> 48000.
        let pos = (o as u64 * FRY_RATE) as f64 / OUT_RATE as f64;
        let i0 = pos as usize;
        let frac = pos - i0 as f64;
        let s0 = *mono.get(i0).unwrap_or(&0);
        let s1 = *mono.get(i0 + 1).unwrap_or(&s0);
        let interp = s0 as f64 + (s1 as f64 - s0 as f64) * frac;
        // Attenuate (~1/2) for a comfortable but clearly audible level.
        let s = (interp as i32 / 2) as i16;
        let b = s.to_le_bytes();
        block[bi] = b[0];
        block[bi + 1] = b[1];
        block[bi + 2] = b[0]; // duplicate to right channel
        block[bi + 3] = b[1];
        bi += 4;
        if bi == BLOCK {
            if !write_block_blocking(device, &block) { return; }
            bi = 0;
        }
    }
    if bi > 0 {
        for k in bi..BLOCK { block[k] = 0; }
        let _ = write_block_blocking(device, &block);
    }

    // Fill the WHOLE ring (32 blocks) + margin with silence so the DMA can't
    // replay leftover clip audio while we wait for the next keystroke.
    let silence = [0u8; BLOCK];
    for _ in 0..40 {
        if !write_block_blocking(device, &silence) { break; }
    }
    hda::delay_us(150_000);
    crate::flog!("speak_fry: done ({} blocks of audio streamed)", out_frames / (BLOCK / 4));
    crate::hda::filelog::flush();
}

/// Play a sine wave tone for a given duration and frequency.
/// Format: 48000 Hz, 16-bit signed PCM, stereo (mandatory HDA format per spec).
fn play_short_tone(device: &mut hda::IntelHDA, duration_secs: f64, freq: f64) {
    const SAMPLE_RATE: f64 = 48000.0;
    const AMPLITUDE: f64 = 3000.0;     // ~9% volume (Framework speaker buzzes loud)
    const CHANNELS: usize = 2;
    const BYTES_PER_SAMPLE: usize = 2;  // 16-bit
    const FRAME_SIZE: usize = CHANNELS * BYTES_PER_SAMPLE; // 4 bytes
    const BLOCK_SIZE: usize = 2048;     // must match SUB_BUFF_SIZE
    const FRAMES_PER_BLOCK: usize = BLOCK_SIZE / FRAME_SIZE; // 512

    let total_frames = (SAMPLE_RATE * duration_secs) as usize;
    let total_blocks = (total_frames + FRAMES_PER_BLOCK - 1) / FRAMES_PER_BLOCK;

    let mut buf = [0u8; BLOCK_SIZE];
    let mut sample_offset: usize = 0;

    for block in 0..total_blocks {
        // Fill the block with sine wave samples
        for frame in 0..FRAMES_PER_BLOCK {
            let t = (sample_offset + frame) as f64 / SAMPLE_RATE;
            let value = libm::sin(2.0 * core::f64::consts::PI * freq * t) * AMPLITUDE;
            let sample = value as i16;
            let bytes = sample.to_le_bytes();

            let byte_offset = frame * FRAME_SIZE;
            // Left channel
            buf[byte_offset] = bytes[0];
            buf[byte_offset + 1] = bytes[1];
            // Right channel (same as left)
            buf[byte_offset + 2] = bytes[0];
            buf[byte_offset + 3] = bytes[1];
        }
        sample_offset += FRAMES_PER_BLOCK;

        // Write block, retrying if the buffer is full
        let mut retries = 0u32;
        loop {
            match device.write_to_output(0, &buf) {
                Ok(Some(_)) => break,
                Ok(None) => {
                    retries += 1;
                    if retries > 50000 {
                        println!("  DMA stalled at block {}/{}, aborting tone", block, total_blocks);
                        return;
                    }
                    hda::delay_us(500);
                }
                Err(e) => {
                    println!("  Audio write error: {:?}", e);
                    return;
                }
            }
        }
    }

    // Overwrite the ENTIRE cyclic ring (32 blocks) with silence so the DMA
    // can't replay leftover tone samples after we stop (the "end blip"). +4
    // margin for the producer staying ahead of the DMA read pointer.
    let silence = [0u8; BLOCK_SIZE];
    for _ in 0..36 {
        let mut retries = 0u32;
        loop {
            match device.write_to_output(0, &silence) {
                Ok(Some(_)) => break,
                Ok(None) => {
                    retries += 1;
                    if retries > 10000 {
                        break;
                    }
                    hda::delay_us(500);
                }
                Err(_) => break,
            }
        }
    }

    // Wait for remaining buffered audio to play out
    hda::delay_us(400_000);
}

////#![deny(warnings)]
//#![feature(int_roundings)]
//
//extern crate bitflags;
//extern crate spin;
//extern crate syscall;
//extern crate event;
//
//use std::convert::TryFrom;
//use std::usize;
//use std::fs::File;
//use std::io::{ErrorKind, Read, Write, Result};
//use std::os::unix::io::{AsRawFd, FromRawFd, RawFd};
//use syscall::{Packet, SchemeBlockMut, EventFlags};
//use std::cell::RefCell;
//use std::sync::Arc;
//
//use event::EventQueue;
//use pcid::{MsiSetFeatureInfo, PcidServerHandle, PciFeature, PciFeatureInfo, SetFeatureInfo};
//use pcid::irq_helpers::{read_bsp_apic_id, allocate_single_interrupt_vector};
//use redox_log::{OutputBuilder, RedoxLogger};
//
//pub mod hda;
//
//                 VEND:PROD
//    Virtualbox   8086:2668
//    QEMU ICH9    8086:293E
//    82801H ICH8  8086:284B
//*/
//
//fn daemon(daemon: redox_daemon::Daemon) -> ! {
//		let mut event_queue = EventQueue::<usize>::new().expect("ihdad: Could not create event queue.");
//
//        syscall::setrens(0, 0).expect("ihdad: failed to enter null namespace");
//
//		let todo = Arc::new(RefCell::new(Vec::<Packet>::new()));
//
//		let todo_irq = todo.clone();
//		let device_irq = device.clone();
//		let socket_irq = socket.clone();
//		event_queue.add(irq_file.as_raw_fd(), move |_event| -> Result<Option<usize>> {
//			let mut irq = [0; 8];
//			irq_file.read(&mut irq)?;
//
//			if device_irq.borrow_mut().irq() {
//				irq_file.write(&mut irq)?;
//
//				let mut todo = todo_irq.borrow_mut();
//				let mut i = 0;
//				while i < todo.len() {
//					if let Some(a) = device_irq.borrow_mut().handle(&mut todo[i]) {
//	                    let mut packet = todo.remove(i);
//	                    packet.a = a;
//						socket_irq.borrow_mut().write(&packet)?;
//	                } else {
//	                    i += 1;
//					}
//				}
//
//				/*
//				let next_read = device_irq.next_read();
//				if next_read > 0 {
//					return Ok(Some(next_read));
//				}
//				*/
//			}
//			Ok(None)
//		}).expect("ihdad: failed to catch events on IRQ file");
//		let socket_fd = socket.borrow().as_raw_fd();
//		let socket_packet = socket.clone();
//		event_queue.add(socket_fd, move |_event| -> Result<Option<usize>> {
//			loop {
//				let mut packet = Packet::default();
//				match socket_packet.borrow_mut().read(&mut packet) {
//		            Ok(0) => return Ok(Some(0)),
//		            Ok(_) => (),
//		            Err(err) => if err.kind() == ErrorKind::WouldBlock {
//		                break;
//		            } else {
//		                return Err(err);
//		            }
//				}
//
//				if let Some(a) = device.borrow_mut().handle(&mut packet) {
//					packet.a = a;
//					socket_packet.borrow_mut().write(&packet)?;
//				} else {
//					todo.borrow_mut().push(packet);
//				}
//			}
//
//			/*
//			let next_read = device.borrow().next_read();
//			if next_read > 0 {
//				return Ok(Some(next_read));
//			}
//			*/
//
//			Ok(None)
//		}).expect("ihdad: failed to catch events on IRQ file");
//
//		for event_count in event_queue.trigger_all(event::Event {
//			fd: 0,
//			flags: EventFlags::empty(),
//		}).expect("ihdad: failed to trigger events") {
//			socket.borrow_mut().write(&Packet {
//				id: 0,
//				pid: 0,
//				uid: 0,
//				gid: 0,
//				a: syscall::number::SYS_FEVENT,
//				b: 0,
//				c: syscall::flag::EVENT_READ.bits(),
//				d: event_count
//			}).expect("ihdad: failed to write event");
//		}
//
//		loop {
//			{
//				//device_loop.borrow_mut().handle_interrupts();
//			}
//			let event_count = event_queue.run().expect("ihdad: failed to handle events");
//			if event_count == 0 {
//				//TODO: Handle todo
//				break;
//			}
//
//			socket.borrow_mut().write(&Packet {
//				id: 0,
//				pid: 0,
//				uid: 0,
//				gid: 0,
//				a: syscall::number::SYS_FEVENT,
//				b: 0,
//				c: syscall::flag::EVENT_READ.bits(),
//				d: event_count
//			}).expect("ihdad: failed to write event");
//		}
//	}
//
//    std::process::exit(0);
//}
//
//fn main() {
//	// Daemonize
//    redox_daemon::Daemon::new(daemon).expect("ihdad: failed to daemonize");
//}
use std::fs::File;
use std::process::Command;
use std::sync::{Arc, Mutex};
use std::i64;

use log::{trace, warn};

use pcid::config::Config;

use pcid::pci::{
    Pci, PciBar, PciBus, PciClass, PciDev, PciFunc, PciHeader, PciHeaderError,
    PciHeaderType,
};
use pcid::pcie::Pcie;

use pcid::config;
use pcid::driver_interface;
use pcid::pci;


//#[derive(StructOpt)]
//#[structopt(about)]
//struct Args {
//    #[structopt(short, long,
//        help="Increase logging level once for each arg.", parse(from_occurrences))]
//    verbose: u8,
//
//    #[structopt(
//        help="A path to a pcid config file or a directory that contains pcid config files.")]
//    config_path: Option<String>,
//}
//

use crossbeam::channel::{Sender, Receiver, bounded};

fn handle_parsed_header(
    state: Arc<State>,
    config: &Config,
    bus_num: u8,
    dev_num: u8,
    func_num: u8,
    header: PciHeader,
    _pcid_to_write: Sender<Vec<u8>>,
    _pcid_from_read: Receiver<Vec<u8>>
) -> Vec<(DriverHandler, SubdriverArguments)> {
    let pci = state.preferred_cfg_access();

    let raw_class: u8 = header.class().into();
    let mut string = format!(
        "PCI {:>02X}/{:>02X}/{:>02X} {:>04X}:{:>04X} {:>02X}.{:>02X}.{:>02X}.{:>02X} {:?}",
        bus_num,
        dev_num,
        func_num,
        header.vendor_id(),
        header.device_id(),
        raw_class,
        header.subclass(),
        header.interface(),
        header.revision(),
        header.class()
    );
    match header.class() {
        PciClass::Legacy if header.subclass() == 1 => string.push_str("  VGA CTL"),
        PciClass::Storage => match header.subclass() {
            0x01 => {
                string.push_str(" IDE");
            }
            0x06 => {
                if header.interface() == 0 {
                    string.push_str(" SATA VND");
                } else if header.interface() == 1 {
                    string.push_str(" SATA AHCI");
                }
            }
            _ => (),
        },
        PciClass::SerialBus => match header.subclass() {
            0x03 => match header.interface() {
                0x00 => {
                    string.push_str(" UHCI");
                }
                0x10 => {
                    string.push_str(" OHCI");
                }
                0x20 => {
                    string.push_str(" EHCI");
                }
                0x30 => {
                    string.push_str(" XHCI");
                }
                _ => (),
            },
            _ => (),
        },
        _ => (),
    }

    for (i, bar) in header.bars().iter().enumerate() {
        if !bar.is_none() {
            string.push_str(&format!(" {}={}", i, bar));
        }
    }

    std::println!("STRING: {}", string);
    let mut driver_handlers = Vec::new();

    for driver in config.drivers.iter() {
        if let Some(class) = driver.class {
            if class != raw_class {
                continue;
            }
        }

        if let Some(subclass) = driver.subclass {
            if subclass != header.subclass() {
                continue;
            }
        }

        if let Some(interface) = driver.interface {
            if interface != header.interface() {
                continue;
            }
        }

        if let Some(ref ids) = driver.ids {
            let mut device_found = false;
            for (vendor, devices) in ids {
                let vendor_without_prefix = vendor.trim_start_matches("0x");
                let vendor = i64::from_str_radix(vendor_without_prefix, 16).unwrap() as u16;

                if vendor != header.vendor_id() {
                    continue;
                }

                for device in devices {
                    if *device == header.device_id() {
                        device_found = true;
                        break;
                    }
                }
            }
            if !device_found {
                continue;
            }
        } else {
            if let Some(vendor) = driver.vendor {
                if vendor != header.vendor_id() {
                    continue;
                }
            }

            if let Some(device) = driver.device {
                if device != header.device_id() {
                    continue;
                }
            }
        }

        if let Some(ref device_id_range) = driver.device_id_range {
            if header.device_id() < device_id_range.start
                || device_id_range.end <= header.device_id()
            {
                continue;
            }
        }

        if let Some(ref args) = driver.command {
            // Enable bus mastering, memory space, and I/O space
            unsafe {
                let mut data = pci.read(bus_num, dev_num, func_num, 0x04);
                data |= 7;
                pci.write(bus_num, dev_num, func_num, 0x04, data);
            }

            // Vendor-specific PCIe snoop / traffic class configuration.
            // Without this, DMA is not cache-coherent on some controllers.
            let vendor_id = header.vendor_id();
            unsafe {
                match vendor_id {
                    // Intel: Set Traffic Class Select (TCSEL) register
                    // PCI config offset 0x44, clear bits [2:0] to select TC0
                    0x8086 => {
                        let tcsel = pci.read(bus_num, dev_num, func_num, 0x44);
                        pci.write(bus_num, dev_num, func_num, 0x44, tcsel & 0xFFFF_FFF8);
                        println!("HDA: Intel TCSEL configured (was {:#X})", tcsel & 0x7);
                    }
                    // AMD: Enable PCIe snoop at PCI config register 0x42
                    // Read byte at 0x42, mask with 0xF8, OR with 0x02 to enable snoop
                    // 0x1022 = AMD chipset HDA, 0x1002 = AMD/ATI GPU HDMI audio
                    0x1022 | 0x1002 => {
                        let reg40 = pci.read(bus_num, dev_num, func_num, 0x40);
                        let byte42 = ((reg40 >> 16) & 0xFF) as u32;
                        let new_byte42 = (byte42 & 0xF8) | 0x02;
                        let new_reg40 = (reg40 & 0xFF00FFFF) | (new_byte42 << 16);
                        pci.write(bus_num, dev_num, func_num, 0x40, new_reg40);
                        println!("HDA: AMD PCIe snoop enabled (reg42: {:#X} -> {:#X})", byte42, new_byte42);
                    }
                    // Nvidia: reg 0x4E, mask 0xF0, enable 0x0F
                    0x10DE => {
                        let reg4c = pci.read(bus_num, dev_num, func_num, 0x4C);
                        let byte4e = ((reg4c >> 16) & 0xFF) as u32;
                        let new_byte4e = (byte4e & 0xF0) | 0x0F;
                        let new_reg4c = (reg4c & 0xFF00FFFF) | (new_byte4e << 16);
                        pci.write(bus_num, dev_num, func_num, 0x4C, new_reg4c);
                        println!("HDA: Nvidia snoop enabled (reg4E: {:#X} -> {:#X})", byte4e, new_byte4e);
                    }
                    _ => {
                        println!("HDA: Unknown vendor {:#06X}, no snoop config applied", vendor_id);
                    }
                }
            }

            // Verify bus master enable stuck, and force cache-coherent DMA by
            // clearing the PCIe "No Snoop Enable" bit (DevCtl bit 11). With
            // NoSnoop disabled the controller's DMA must be snooped, so device
            // DMA stays coherent with CPU caches without relying on clflush.
            unsafe {
                let cmd = pci.read(bus_num, dev_num, func_num, 0x04);
                crate::flog!("PCI {:02X}:{:02X}.{:X} vend={:#06X} dev={:#06X} cmd={:#06X} BusMaster={}",
                    bus_num, dev_num, func_num, header.vendor_id(), header.device_id(),
                    cmd & 0xFFFF, (cmd >> 2) & 1);

                let status = (pci.read(bus_num, dev_num, func_num, 0x04) >> 16) as u16;
                if status & (1 << 4) != 0 {
                    let mut cap_ptr = (pci.read(bus_num, dev_num, func_num, 0x34) & 0xFC) as u8;
                    let mut guard = 0;
                    while cap_ptr >= 0x40 && guard < 48 {
                        let capw = pci.read(bus_num, dev_num, func_num, cap_ptr as u16);
                        let cap_id = (capw & 0xFF) as u8;
                        let next = ((capw >> 8) & 0xFF) as u8;
                        if cap_id == 0x10 {
                            // PCI Express capability: DevCtl is 16 bits at cap+0x08.
                            let devctl_off = cap_ptr as u16 + 0x08;
                            let dc = pci.read(bus_num, dev_num, func_num, devctl_off);
                            let devctl = (dc & 0xFFFF) as u16;
                            let new_devctl = devctl & !(1 << 11); // clear No Snoop Enable
                            pci.write(bus_num, dev_num, func_num, devctl_off,
                                (dc & 0xFFFF_0000) | new_devctl as u32);
                            let rb = (pci.read(bus_num, dev_num, func_num, devctl_off) & 0xFFFF) as u16;
                            crate::flog!("  PCIe cap@{:#X} DevCtl {:#06X} -> {:#06X} readback {:#06X} (cleared NoSnoop)",
                                cap_ptr, devctl, new_devctl, rb);
                        }
                        cap_ptr = next;
                        guard += 1;
                    }
                }
            }
            crate::hda::filelog::flush();

            // Set IRQ line to 9 if not set
            let mut irq;
            let interrupt_pin;

            unsafe {
                let mut data = pci.read(bus_num, dev_num, func_num, 0x3C);
                irq = (data & 0xFF) as u8;
                interrupt_pin = ((data & 0x0000_FF00) >> 8) as u8;
                if irq == 0xFF {
                    irq = 9;
                }
                data = (data & 0xFFFFFF00) | irq as u32;
                pci.write(bus_num, dev_num, func_num, 0x3C, data);
            };

            // Find BAR sizes
            //TODO: support 64-bit BAR sizes?
            let mut bars = [PciBar::None; 6];
            let mut bar_sizes = [0; 6];
            unsafe {
                let count = match header.header_type() {
                    PciHeaderType::GENERAL => 6,
                    PciHeaderType::PCITOPCI => 2,
                    _ => 0,
                };

                for i in 0..count {
                    bars[i] = header.get_bar(i);

                    let offset = 0x10 + (i as u8) * 4;

                    let original = pci.read(bus_num, dev_num, func_num, offset.into());
                    pci.write(bus_num, dev_num, func_num, offset.into(), 0xFFFFFFFF);

                    let new = pci.read(bus_num, dev_num, func_num, offset.into());
                    pci.write(bus_num, dev_num, func_num, offset.into(), original);

                    let masked = if new & 1 == 1 {
                        new & 0xFFFFFFFC
                    } else {
                        new & 0xFFFFFFF0
                    };

                    let size = (!masked).wrapping_add(1);
                    bar_sizes[i] = if size <= 1 { 0 } else { size };
                }
            }

            let capabilities = if header.status() & (1 << 4) != 0 {
                let bus = PciBus {
                    pci: state.preferred_cfg_access(),
                    num: bus_num,
                };
                let dev = PciDev {
                    bus: &bus,
                    num: dev_num,
                };
                let func = PciFunc {
                    dev: &dev,
                    num: func_num,
                };
                CapabilitiesIter {
                    inner: CapabilityOffsetsIter::new(header.cap_pointer(), &func),
                }
                .collect::<Vec<_>>()
            } else {
                Vec::new()
            };
            use driver_interface::LegacyInterruptPin;

            let legacy_interrupt_pin = match interrupt_pin {
                0 => None,
                1 => Some(LegacyInterruptPin::IntA),
                2 => Some(LegacyInterruptPin::IntB),
                3 => Some(LegacyInterruptPin::IntC),
                4 => Some(LegacyInterruptPin::IntD),

                other => {
                    warn!("pcid: invalid interrupt pin: {}", other);
                    None
                }
            };

            let func = driver_interface::PciFunction {
                bars,
                bar_sizes,
                bus_num,
                dev_num,
                func_num,
                devid: header.device_id(),
                legacy_interrupt_line: irq,
                legacy_interrupt_pin,
                venid: header.vendor_id(),
            };

            let subdriver_args = driver_interface::SubdriverArguments { func };

            let mut args = args.iter();
            if let Some(program) = args.next() {
                let mut command = Command::new(program);
                for arg in args {
                    let arg = match arg.as_str() {
                        "$BUS" => format!("{:>02X}", bus_num),
                        "$DEV" => format!("{:>02X}", dev_num),
                        "$FUNC" => format!("{:>02X}", func_num),
                        "$NAME" => func.name(),
                        "$BAR0" => format!("{}", bars[0]),
                        "$BAR1" => format!("{}", bars[1]),
                        "$BAR2" => format!("{}", bars[2]),
                        "$BAR3" => format!("{}", bars[3]),
                        "$BAR4" => format!("{}", bars[4]),
                        "$BAR5" => format!("{}", bars[5]),
                        "$BARSIZE0" => format!("{:>08X}", bar_sizes[0]),
                        "$BARSIZE1" => format!("{:>08X}", bar_sizes[1]),
                        "$BARSIZE2" => format!("{:>08X}", bar_sizes[2]),
                        "$BARSIZE3" => format!("{:>08X}", bar_sizes[3]),
                        "$BARSIZE4" => format!("{:>08X}", bar_sizes[4]),
                        "$BARSIZE5" => format!("{:>08X}", bar_sizes[5]),
                        "$IRQ" => format!("{}", irq),
                        "$VENID" => format!("{:>04X}", header.vendor_id()),
                        "$DEVID" => format!("{:>04X}", header.device_id()),
                        _ => arg.clone(),
                    };
                    command.arg(&arg);
                }

                std::println!("PCID SPAWN {:?}", command);

                //TODO:
                //this is RedoxOS's way of doing shared memory
                //two raw file descriptors,
                //reader on one side
                //writer on the other, but attached via the same
                //file descriptor.
                //the TO/FROM are id 1/2
                //then there are two references handed out
                //one to the local handle_spawn function
                //and the other to the "global" envrionemtn variable space
                //with the TO_READ/FROM_WRITE being shared
                //and TO_WRITE/FROM_READ being moved to a local spawned thread
                //to configure HDAudio, we will need to run this function using shared memory, ideally Arc<Mutex<Vec<u8>>> since this creates shared, mutally excluusive access to what can become raw buffers (&mut [u8])
                //there will need to be two buffers, then you can pass copies of the Arc<_> to the two sides.
                //they can unlock them when needed
                //hopefully locking doesn't cause problems with timing.
                // Redox also wants to set envrionemnt variables ( as set in handle_spanw
                // and use that to launch a separate command
                // in this case, it actually looks up a driver's `main` to run.
                // it seems every driver runs in its own process.
                // we need to combine everyting to run in one binary.
                // how to share all these variables, and launch with all these options, I'm not really sure....
                // so here we'd want to find `hdaudio` outputs, and then figure out how to pass all these arguments
                // also, microkernal for the win, I never realized how clean it was to run stuff like this in a uKernal environment.
                // I'd be interested on adding development to this platform in the future
                // maybe get atspi going? with help from the main developer? (remember he is in the Odilia channel)

                // try to hook into spawning a separate function but not a separate command

                // TODO: fix command issues
                        let driver_handler = DriverHandler {
                            bus_num,
                            dev_num,
                            func_num,
                            config: driver.clone(),
                            header,
                            state: Arc::clone(&state),
                            capabilities,
                        };
                        driver_handlers.push((driver_handler, subdriver_args));
                        /*
                        driver_handler.handle_spawn(
                            &mut pcid_to_write.clone(),
                            &mut pcid_from_read.clone(),
                            subdriver_args,
                        );
                        */
            }
        }
    }
  driver_handlers
}

use crate::config::DriverConfig;

fn pci_main(
  pcid_from_write: &mut Sender<Vec<u8>>,
  pcid_to_read: &mut Receiver<Vec<u8>>
) -> Vec<(DriverHandler, SubdriverArguments)> {
    //let config = toml::from_str(include_str!("../config.toml")).unwrap_or(Config::default());
    let config_hda = DriverConfig {
      name: Some("Intel HD Audio".to_string()),
      class: Some(4),
      subclass: Some(3),
      command: Some(vec!["ihdad".to_string()]),
      use_channel: Some(true),
      interface: None,
      ids: None,
      vendor: None,
      device: None,
      device_id_range: None,
    };
    let mut config = Config::default();
    config.drivers.push(config_hda);

    let pci = Arc::new(Pci::new());

    let state = Arc::new(State {
        pci: Arc::clone(&pci),
        pcie: match Pcie::new(Arc::clone(&pci)) {
            Ok(pcie) => Some(pcie),
            Err(error) => {
                std::println!("Couldn't retrieve PCIe info, perhaps the kernel is not compiled with acpi? Using the PCI 3.0 configuration space instead. Error: {:?}", error);
                None
            }
        },
        threads: Mutex::new(Vec::new()),
    });

    let pci = state.preferred_cfg_access();

    std::println!("PCI BS/DV/FN VEND:DEVI CL.SC.IN.RV");
    let mut handlers = Vec::new();

    let mut bus_nums = vec![0];
    let mut bus_i = 0;
    'bus: while bus_i < bus_nums.len() {
        let bus_num = bus_nums[bus_i];
        bus_i += 1;

        let bus = PciBus { pci, num: bus_num };
        'dev: for dev in bus.devs() {
            for func in dev.funcs() {
                let func_num = func.num;
                match PciHeader::from_reader(func) {
                    Ok(header) => {
                        handle_parsed_header(
                            Arc::clone(&state),
                            &config,
                            bus.num,
                            dev.num,
                            func_num,
                            header,
                            pcid_from_write.clone(),
                            pcid_to_read.clone(),
                        ).into_iter()
                        .for_each(|d| handlers.push(d));
                        if let PciHeader::PciToPci {
                            secondary_bus_num, ..
                        } = header
                        {
                            bus_nums.push(secondary_bus_num);
                        }
                    }
                    Err(PciHeaderError::NoDevice) => {
                        if func_num == 0 {
                            if dev.num == 0 {
                                trace!("PCI {:>02X}: no bus", bus.num);
                                continue 'bus;
                            } else {
                                trace!("PCI {:>02X}/{:>02X}: no dev", bus.num, dev.num);
                                continue 'dev;
                            }
                        }
                    }
                    Err(PciHeaderError::UnknownHeaderType(id)) => {
                        warn!("pcid: unknown header type: {}", id);
                    }
                }
            }
        }
    }

  // --- AMD-Vi IOMMU probe (read-only) ---
  // Bus master is on, NoSnoop cleared, addresses are identity-mapped, yet the
  // controller's DMA never reaches CPU memory. The remaining intercept is the
  // IOMMU (PCI 00:00.2). Read its control register to confirm the firmware
  // left it enabled in pre-boot.
  unsafe {
    let id0 = pci.read(0, 0, 2, 0x00);
    let vend = (id0 & 0xFFFF) as u16;
    let devid = ((id0 >> 16) & 0xFFFF) as u16;
    crate::flog!("IOMMU probe 00:00.2 vendor={:#06X} device={:#06X}", vend, devid);
    if vend == 0x1022 {
      let mut cap = (pci.read(0, 0, 2, 0x34) & 0xFC) as u8;
      let mut guard = 0;
      while cap >= 0x40 && guard < 48 {
        let capw = pci.read(0, 0, 2, cap as u16);
        let id = (capw & 0xFF) as u8;
        let next = ((capw >> 8) & 0xFF) as u8;
        crate::flog!("  cap@{:#04X} id={:#04X}", cap, id);
        if id == 0x0F {
          // AMD IOMMU capability: MMIO base in cap+0x04 (low, bit0=enable) / cap+0x08 (high)
          let base_lo = pci.read(0, 0, 2, cap as u16 + 0x04);
          let base_hi = pci.read(0, 0, 2, cap as u16 + 0x08);
          let en = base_lo & 1;
          let mmio = ((base_hi as u64) << 32) | ((base_lo & 0xFFFF_C000) as u64);
          crate::flog!("  IOMMU cap base_lo={:#010X} base_hi={:#010X} mmio={:#X} mmio_en={}",
            base_lo, base_hi, mmio, en);
          crate::hda::filelog::flush();
          if en == 1 && mmio >= 0x1000 {
            // IOMMU MMIO: Device Table Base @ +0x00, Control Register @ +0x18 (64-bit)
            let devtab = core::ptr::read_volatile((mmio + 0x00) as *const u64);
            let ctrl = core::ptr::read_volatile((mmio + 0x18) as *const u64);
            crate::flog!("  IOMMU DevTabBase={:#018X} Control={:#018X} IommuEn={}",
              devtab, ctrl, ctrl & 1);
            // NOTE: do NOT clear IommuEn here — writing the control register
            // HANGS this firmware. We cooperate with the IOMMU via
            // EFI_PCI_IO_PROTOCOL.Map() instead (see efidma module).
          } else {
            crate::flog!("  IOMMU MMIO not enabled or base invalid; not touching control reg");
          }
        }
        cap = next;
        guard += 1;
      }
    }
  }
  crate::hda::filelog::flush();

  handlers
}
