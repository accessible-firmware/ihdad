//! A `std::io::{Result, Error, ErrorKind}`-shaped surface that exists in
//! both build modes.
//!
//! Under `--features std` these are literal re-exports of `std::io`; the
//! historical code paths (incl. error formatting via `Display`) are
//! untouched. Under `no_std` we provide a `Result<T, Error>` with a
//! `kind()` and a static-str `description`, which is everything the
//! library actually uses — no `std::io::Read`/`Write` traits.
//!
//! The crate-wide convention is: anywhere a function returns
//! `std::io::Result`, change the path to `crate::io_compat::Result`.
//! Same for `Error` / `ErrorKind`. All `Error::new(kind, "msg")` calls
//! continue to work; in no_std mode the `msg` is required to be a
//! `&'static str` (we have no allocator at error-construction time
//! reliably for an `Into<Box<dyn Error>>` argument).
//!
//! In std mode `crate::io_compat::Result<T>` is identical to
//! `std::io::Result<T>`, so existing `?` propagation works unchanged.

#[cfg(feature = "std")]
pub use std::io::{Error, ErrorKind, Result};

#[cfg(not(feature = "std"))]
pub type Result<T> = core::result::Result<T, Error>;

#[cfg(not(feature = "std"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorKind {
    NotFound,
    PermissionDenied,
    ConnectionRefused,
    ConnectionReset,
    ConnectionAborted,
    NotConnected,
    AddrInUse,
    AddrNotAvailable,
    BrokenPipe,
    AlreadyExists,
    WouldBlock,
    InvalidInput,
    InvalidData,
    TimedOut,
    WriteZero,
    Interrupted,
    Unsupported,
    UnexpectedEof,
    OutOfMemory,
    Other,
}

#[cfg(not(feature = "std"))]
#[derive(Debug, Clone, Copy)]
pub struct Error {
    kind: ErrorKind,
    msg: &'static str,
}

#[cfg(not(feature = "std"))]
impl Error {
    /// Match `std::io::Error::new` for source-compat. In no_std the second
    /// argument is constrained to `&'static str` instead of
    /// `Into<Box<dyn std::error::Error>>` — every existing call site in
    /// this crate already passes a string literal, so the difference is
    /// invisible.
    pub fn new(kind: ErrorKind, msg: &'static str) -> Self {
        Self { kind, msg }
    }

    pub fn kind(&self) -> ErrorKind {
        self.kind
    }

    /// Static message string this error was constructed with.
    pub fn message(&self) -> &'static str {
        self.msg
    }
}

#[cfg(not(feature = "std"))]
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.msg)
    }
}
