//! Interface to `pcid`.
//!
//! The `driver_interface` and `config` submodules are inherited from the
//! Redox-side pcid daemon. They depend on `std::sync::Mutex`/`std::thread`/
//! `std::env`/IPC via crossbeam channels — none of which exist in `no_std`,
//! and none of which the GRUB module path actually uses (it talks to HDA
//! controllers directly through `pci` + `pcie` + the high-level `api`
//! module). They're therefore gated behind the `std` feature; the
//! standalone `ihdad.efi` binary (also `std`-gated via the `binary`
//! feature) still sees them.

pub mod pci;
pub mod pcie;

#[cfg(feature = "std")]
pub mod driver_interface;
#[cfg(feature = "std")]
pub mod config;
#[cfg(feature = "std")]
pub use driver_interface::*;
