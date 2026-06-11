//! `std::sync::{Mutex, Once}` shims that work in both build modes.
//!
//! The `std` build re-exports the real `std::sync::Mutex`/`Once` — the
//! existing call sites (`m.lock().unwrap()`, `o.call_once(|| ...)`) keep
//! their exact semantics, including poisoning. That's deliberate: the
//! standalone `ihdad.efi` binary is the one piece of this crate that's
//! actually been tested on real hardware, and the user's hardware-test
//! workflow validates equivalence to the prior `std::sync` behaviour.
//!
//! The `no_std` build provides API-compatible wrappers around
//! `spin::Mutex` and `spin::Once`. spin doesn't have poisoning, so
//! `lock()` returns a `Result` that's always `Ok` — this keeps the
//! `m.lock().unwrap()` call pattern compiling unchanged. `Once::new`
//! and `Once::call_once` match spin's existing API directly.
//!
//! Only the two `pcid` PCI-config-access modules use these; everything
//! else in the library path that needs mutual exclusion (e.g.
//! `hda::device::IntelHDA::handles`) already imported `spin::Mutex`
//! directly, so this compat layer covers exactly two import sites.

#[cfg(feature = "std")]
pub use std::sync::{Mutex, MutexGuard, Once};

#[cfg(not(feature = "std"))]
pub use no_std_impl::{Mutex, MutexGuard, Once};

#[cfg(not(feature = "std"))]
mod no_std_impl {
    use core::ops::{Deref, DerefMut};

    /// `std::sync::Mutex`-shaped wrapper around `spin::Mutex`. `lock()`
    /// returns `Result<Guard, PoisonError<Guard>>` for source-compat with
    /// the std build, but spin can't poison so the error variant is
    /// uninhabited from the runtime side — callers get `Ok` always.
    pub struct Mutex<T: ?Sized> {
        inner: spin::Mutex<T>,
    }

    impl<T> Mutex<T> {
        pub const fn new(t: T) -> Self {
            Self { inner: spin::Mutex::new(t) }
        }
    }

    impl<T: ?Sized> Mutex<T> {
        /// Matches std's signature — returns a `Result` so existing
        /// `.lock().unwrap()` call patterns keep compiling.
        pub fn lock(&self) -> Result<MutexGuard<'_, T>, core::convert::Infallible> {
            Ok(MutexGuard { inner: self.inner.lock() })
        }
    }

    /// `std::sync::MutexGuard` lookalike around `spin::MutexGuard`.
    pub struct MutexGuard<'a, T: ?Sized + 'a> {
        inner: spin::MutexGuard<'a, T>,
    }

    impl<'a, T: ?Sized> Deref for MutexGuard<'a, T> {
        type Target = T;
        fn deref(&self) -> &T { &*self.inner }
    }

    impl<'a, T: ?Sized> DerefMut for MutexGuard<'a, T> {
        fn deref_mut(&mut self) -> &mut T { &mut *self.inner }
    }

    /// `std::sync::Once`-shaped wrapper around `spin::Once`. spin's `Once`
    /// stores a value, but the lib path only uses it for its lazy-init
    /// side effect via `call_once`, which has the same signature in both.
    pub struct Once {
        inner: spin::Once<()>,
    }

    impl Once {
        pub const fn new() -> Self {
            Self { inner: spin::Once::new() }
        }

        pub fn call_once<F: FnOnce()>(&self, f: F) {
            self.inner.call_once(|| { f(); });
        }
    }
}
