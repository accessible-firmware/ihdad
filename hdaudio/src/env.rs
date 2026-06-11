//! Bridge to the host environment.
//!
//! Two cases:
//!
//! 1. **`std` feature on**: this crate runs as the standalone `ihdad.efi`
//!    UEFI application. The EFI System Table and Image Handle are
//!    reachable through `std::os::uefi::env::{system_table, image_handle}`,
//!    and diagnostic output goes to the UEFI console via `std::println!`.
//!    This module's functions just forward to those.
//!
//! 2. **`std` feature off (no_std)**: this crate is statically linked into
//!    another freestanding binary (e.g. a GRUB module) that has its own
//!    runtime. Before any code path that touches the EFI globals runs, the
//!    consumer must call [`set_efi_globals`] with the System Table and
//!    Image Handle pointers it received from its own loader. Optionally it
//!    can also register a log sink via [`set_log`] so that the `ihd_println!`
//!    family stops being silent; on GRUB this hooks into `grub_printf`.
//!
//! The two layers exist so the rest of the crate can call e.g.
//! `env::system_table()` without sprinkling `cfg(feature = "std")` blocks
//! at every site.

use core::ffi::c_void;
use core::ptr::NonNull;
use core::sync::atomic::{AtomicPtr, AtomicUsize, Ordering};

// ---------------------------------------------------------------------------
// EFI globals
// ---------------------------------------------------------------------------

#[cfg(not(feature = "std"))]
static EFI_SYSTEM_TABLE: AtomicPtr<c_void> = AtomicPtr::new(core::ptr::null_mut());
#[cfg(not(feature = "std"))]
static EFI_IMAGE_HANDLE: AtomicPtr<c_void> = AtomicPtr::new(core::ptr::null_mut());

/// Install the EFI System Table and Image Handle that the rest of `ihdad`'s
/// no_std code path should use. A no-op (and unnecessary) when the `std`
/// feature is on — std's own `os::uefi::env` machinery handles it then.
///
/// Safe to call multiple times; the last set values win.
///
/// # Safety
/// The two pointers must remain valid for the lifetime of every call into
/// this crate. In particular UEFI Boot Services must still be live — the
/// HDA controller reset path uses `BootServices.Stall` and the IOMMU path
/// uses `EFI_PCI_IO_PROTOCOL`, both of which evaporate at
/// `ExitBootServices`.
pub unsafe fn set_efi_globals(image_handle: *mut c_void, system_table: *mut c_void) {
    #[cfg(not(feature = "std"))]
    {
        EFI_SYSTEM_TABLE.store(system_table, Ordering::Release);
        EFI_IMAGE_HANDLE.store(image_handle, Ordering::Release);
    }
    #[cfg(feature = "std")]
    {
        // Avoid `unused_variables` without complicating the public signature.
        let _ = (image_handle, system_table);
    }
}

/// Returns the EFI System Table pointer, or null if neither
/// [`set_efi_globals`] (no_std) nor `efi_main` (std) has run yet.
#[inline]
pub fn system_table() -> *mut c_void {
    #[cfg(feature = "std")]
    {
        // `std::os::uefi::env::try_system_table` is `pub(crate)`, but the
        // backing atomic is `pub` (under #[doc(hidden)]). Read it
        // directly so we get the same "may be null before efi_main runs"
        // semantics as the std API, without panicking on access.
        std::os::uefi::env::globals::SYSTEM_TABLE.load(Ordering::Acquire)
    }
    #[cfg(not(feature = "std"))]
    {
        EFI_SYSTEM_TABLE.load(Ordering::Acquire)
    }
}

/// Returns the EFI Image Handle, or null if not set.
#[inline]
pub fn image_handle() -> *mut c_void {
    #[cfg(feature = "std")]
    {
        std::os::uefi::env::globals::IMAGE_HANDLE.load(Ordering::Acquire)
    }
    #[cfg(not(feature = "std"))]
    {
        EFI_IMAGE_HANDLE.load(Ordering::Acquire)
    }
}

/// As above, returned as a `NonNull` for ergonomics when you've just checked
/// it can't be null (e.g. inside an `if let Some(_) = boot_services()`).
#[inline]
pub fn system_table_nonnull() -> Option<NonNull<c_void>> {
    NonNull::new(system_table())
}

// ---------------------------------------------------------------------------
// Log sink
// ---------------------------------------------------------------------------
//
// Stored as a raw function pointer (cast through usize so AtomicPtr's
// covariance over `*mut ()` doesn't trip us up). The signature is:
//
//   extern "C" fn(ptr: *const u8, len: usize)
//
// — the consumer writes those bytes wherever it likes. The contract is
// that the bytes are valid UTF-8 (we format through alloc::format!) and
// not NUL-terminated (the consumer should rely on `len`). On a GRUB
// module this is bridged to grub_printf via a tiny "%.*s" wrapper on
// the C side.

#[cfg(not(feature = "std"))]
static LOG_SINK: AtomicUsize = AtomicUsize::new(0);

/// Type of the log-sink callback an embedder can install via [`set_log`].
pub type LogFn = unsafe extern "C" fn(ptr: *const u8, len: usize);

/// Install a log sink. Called from the embedder during init; passing `None`
/// clears the sink (subsequent log messages are silently dropped).
///
/// No-op when the `std` feature is on — diagnostics go through `std::print!`
/// to the UEFI console instead, by way of the `ihd_println!` macro.
pub fn set_log(sink: Option<LogFn>) {
    #[cfg(not(feature = "std"))]
    {
        let raw = match sink {
            Some(f) => f as usize,
            None => 0usize,
        };
        LOG_SINK.store(raw, Ordering::Release);
    }
    #[cfg(feature = "std")]
    {
        let _ = sink;
    }
}

/// Used by the `ihd_println!` / `ihd_print!` macros in no_std mode. Public
/// because macros expand in the caller's crate; not intended for direct
/// use. The std-mode macros bypass this entirely.
#[cfg(not(feature = "std"))]
#[doc(hidden)]
pub fn log_str(s: &str) {
    let raw = LOG_SINK.load(Ordering::Acquire);
    if raw == 0 {
        return;
    }
    let f: LogFn = unsafe { core::mem::transmute(raw) };
    unsafe { f(s.as_ptr(), s.len()) };
}

/// Trivial no-op stub so std builds can still name `log_str` for symmetry
/// (not used by the std-mode macros, but allows downstream code to refer
/// to it unconditionally if it ever needs to).
#[cfg(feature = "std")]
#[doc(hidden)]
pub fn log_str(_s: &str) {}
