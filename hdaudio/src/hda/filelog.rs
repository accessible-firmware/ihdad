//! File-based logging for UEFI debugging.
//!
//! Buffers log lines in memory, then writes the entire log to `ihdad.log`
//! on flush.
//!
//! After a run, review the file:
//!   - EFI shell: `type ihdad.log`
//!   - Or mount the QEMU disk image from the host

use std::sync::atomic::{AtomicU32, AtomicBool, Ordering};
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
    let line = std::format!("[{:05}] {}\n", seq, msg);
    let mut buf = LOG_BUF.lock();
    buf.extend_from_slice(line.as_bytes());
}

/// Write buffered log data to `ihdad.log`.
/// Only prints to console on success. On failure, dumps log tail to console
/// as fallback so diagnostic data is not lost.
pub fn flush() {
    let buf = LOG_BUF.lock();
    if buf.is_empty() {
        return;
    }
    let len = buf.len();

    // Try several path variants — UEFI path handling varies
    let paths: &[&str] = &["ihdad.log", "\\ihdad.log", ".\\ihdad.log"];
    for path in paths {
        if std::fs::write(path, &*buf).is_ok() {
            println!("IHDAD.LOG OK ({} bytes -> {})", len, path);
            return;
        }
    }

    // All paths failed — dump to console so data is not lost
    println!("=== IHDAD.LOG WRITE FAILED, DUMPING TO CONSOLE ===");
    if let Ok(s) = std::str::from_utf8(&buf) {
        print!("{}", s);
    }
    println!("=== END LOG DUMP ===");
}

/// Convenience macro — works like `println!` but writes to the log buffer.
#[macro_export]
macro_rules! flog {
    ($($arg:tt)*) => {{
        let msg = std::format!($($arg)*);
        $crate::hda::filelog::write_line(&msg);
    }};
}
