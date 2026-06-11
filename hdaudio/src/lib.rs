//! `ihdad` — Intel HD Audio driver, library form.
//!
//! Two layers are exposed:
//!
//! - **Low-level**: the `hda`, `pcid`, and `efidma` modules contain the
//!   raw controller / PCI / IOMMU-DMA machinery used by `main.rs`.
//!   Reach for these when you need full control (custom pin selection,
//!   custom stream formats, codec verb access, …).
//!
//! - **High-level**: the `api` module wraps the common path —
//!   bring up the controller, push PCM through the speaker, then tear
//!   the controller back down cleanly for an OS handoff. Suited to
//!   embedding into bootloaders / BIOS code that just want audible TTS.
//!
//! See `src/main.rs` for a full end-to-end example.
//!
//! ## `std` vs `no_std`
//!
//! The library compiles in two modes:
//!
//! - **Default (no_std + alloc)** — for consumers that need to statically
//!   archive this crate into a freestanding binary that emits ELF (e.g.
//!   the GRUB `speak` module). Errors come back as
//!   [`api::Error`]. Debug output is silent unless the consumer hooks
//!   into [`env::set_log`]. The EFI System Table is provided by the
//!   consumer via [`env::set_efi_globals`] — there's no
//!   `std::os::uefi::env` available.
//!
//! - **`--features std`** — re-enables the historical UEFI-application
//!   path. Errors are `std::io::Result`, `println!` lights up the UEFI
//!   console, and `std::os::uefi::env::system_table()` is used to find
//!   the EFI System Table. This is what `src/main.rs` (the standalone
//!   `ihdad.efi` binary, gated behind the `binary` feature) uses.

#![allow(dead_code)]
#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(feature = "std", feature(uefi_std))]

// `#[macro_use]` so `vec!`, `format!`, `write!`, etc. are reachable without
// per-file `use alloc::format;` imports. In std mode these are already in
// the prelude (and refer to the same macro definitions, so behaviour is
// identical); the attribute matters only for no_std.
#[macro_use]
extern crate alloc;

pub mod hda;
pub mod pcid;
pub mod efidma;
pub mod api;
pub mod env;
pub mod io_compat;
pub mod sync_compat;

pub use api::{
    AudioFormat, native_format, NATIVE_FORMAT,
    HdaController, discover_intel_hda,
    init_intel_hda, init_intel_hda_auto,
    speak_intel_hda, shutdown_intel_hda,
};

pub use hda::IntelHDA;
pub use hda::stream::{
    BitsPerSample, SampleRate,
    SR_8, SR_11_025, SR_16, SR_22_05, SR_32,
    SR_44_1, SR_48, SR_88_1, SR_96, SR_176_4, SR_192,
};

// ---------------------------------------------------------------------------
// Diagnostic-output macros (shared no_std/std)
// ---------------------------------------------------------------------------
//
// The codebase liberally uses `std::println!` for trace output. In std mode
// those calls go to the UEFI console exactly as they always have; in no_std
// mode they route to whatever sink the consumer registered with
// `env::set_log` (typically GRUB's `grub_printf`), or are silently dropped
// if no sink is registered. Using these macros throughout means a single
// `#[cfg]` decision in lib.rs controls debug behaviour, instead of every
// call site needing a feature gate.

/// Like `std::println!` — line-terminated diagnostic output.
#[macro_export]
macro_rules! ihd_println {
    ($($arg:tt)*) => {{
        #[cfg(feature = "std")]
        { std::println!($($arg)*) }
        #[cfg(not(feature = "std"))]
        {
            let s = alloc::format!("{}\n", core::format_args!($($arg)*));
            $crate::env::log_str(&s);
        }
    }};
}

/// Like `std::print!`.
#[macro_export]
macro_rules! ihd_print {
    ($($arg:tt)*) => {{
        #[cfg(feature = "std")]
        { std::print!($($arg)*) }
        #[cfg(not(feature = "std"))]
        {
            let s = alloc::format!("{}", core::format_args!($($arg)*));
            $crate::env::log_str(&s);
        }
    }};
}
