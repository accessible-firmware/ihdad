//! File-based logging for UEFI debugging.
//!
//! Buffers log lines in memory, then writes the entire log to `ihdad.log`
//! on flush.
//!
//! After a run, review the file:
//!   - EFI shell: `type ihdad.log`
//!   - Or mount the QEMU disk image from the host

use alloc::vec::Vec;
use core::sync::atomic::{AtomicU32, AtomicBool, Ordering};
use spin::Mutex;

static LOG_SEQ: AtomicU32 = AtomicU32::new(0);
static LOG_BUF: Mutex<Vec<u8>> = Mutex::new(Vec::new());
static LOG_INIT: AtomicBool = AtomicBool::new(false);

/// Initialize the file logger. Safe to call multiple times.
pub fn init() {
    if LOG_INIT.compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst).is_ok() {
        {
            let mut buf = LOG_BUF.lock();
            buf.reserve(256 * 1024);
        }
        write_line("=== ihdad log started ===");
        // Test flush immediately so user can see if file I/O works
        flush();
    }
}

/// Append a single formatted line to the in-memory log buffer.
pub fn write_line(msg: &str) {
    let seq = LOG_SEQ.fetch_add(1, Ordering::Relaxed);
    // `format!` is the alloc crate's macro, brought into scope crate-wide by
    // `#[macro_use] extern crate alloc;` in lib.rs and main.rs. Identical to
    // `std::format!` in the std build (std re-exports the same macro), so
    // the standalone `ihdad.efi` log output is byte-for-byte unchanged.
    let line = format!("[{:05}] {}\n", seq, msg);
    let mut buf = LOG_BUF.lock();
    buf.extend_from_slice(line.as_bytes());
}

/// Write buffered log data to `ihdad.log`.
/// Only prints to console on success. On failure, dumps log tail to console
/// as fallback so diagnostic data is not lost.
///
/// In `no_std` builds the file-write fallback is gone — there's no host
/// filesystem available when this crate is statically linked into another
/// freestanding binary (e.g. a GRUB module). The buffer still accumulates,
/// and the console-dump branch routes through `ihd_println!` (= the
/// consumer-installed log sink). The `flush` function is therefore not a
/// no-op in no_std mode — it still serves as the "spill everything we've
/// buffered to the log sink right now" call.
pub fn flush() {
    let buf = LOG_BUF.lock();
    if buf.is_empty() {
        return;
    }
    let _len = buf.len();

    #[cfg(feature = "std")]
    {
        // Try several path variants — UEFI path handling varies.
        let paths: &[&str] = &["ihdad.log", "\\ihdad.log", ".\\ihdad.log"];
        for path in paths {
            if std::fs::write(path, &*buf).is_ok() {
                crate::ihd_println!("IHDAD.LOG OK ({} bytes -> {})", _len, path);
                return;
            }
        }
    }

    // All file paths failed (std mode) or skipped entirely (no_std mode) —
    // dump the buffer to the active console / log sink instead so the
    // diagnostic data is not lost.
    crate::ihd_println!("=== IHDAD.LOG WRITE FAILED, DUMPING TO CONSOLE ===");
    if let Ok(s) = core::str::from_utf8(&buf) {
        crate::ihd_print!("{}", s);
    }
    crate::ihd_println!("=== END LOG DUMP ===");
}

/// Snapshot of everything buffered so far. The buffer accumulates across the
/// whole run (it is never cleared), so this returns the complete `flog!`
/// trace — controller reset, codec enumeration, the `DIAG dma-start` line,
/// stream-start attempts, etc. A no_std embedder with no `std::fs` can call
/// this and persist the bytes itself (e.g. write them to the boot ESP), which
/// is the only way to recover the trace on a board where the console isn't
/// usable.
pub fn contents() -> Vec<u8> {
    LOG_BUF.lock().clone()
}

/// Convenience macro — works like `println!` but writes to the log buffer.
#[macro_export]
macro_rules! flog {
    ($($arg:tt)*) => {{
        let msg = alloc::format!($($arg)*);
        $crate::hda::filelog::write_line(&msg);
    }};
}
