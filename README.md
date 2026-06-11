# `ihdad`

An attempt to port the RedoxOS HD Audio driver to pure Rust, with no standard library.
Currently in beta, with basic support for two known audio chipsets.

* [x] `x86_64-unknown-uefi` (nightly, with std)
* [ ] `x86_64-unknown-uefi` (stable, `no_std`)
* [x] `x86_64-unknown-unknown` (`no_std`, stable, possibly `no_alloc`?)

## Running (sine wave)

* `cd ihdad`
* `cargo +nightly build --target x86_64-unknown-uefi`
* `cd ..`
* `./qemu.sh`
* once inside QEMU
* `FS0:`
* `.\ihdad.efi`

## Running (TTS)

* clone the freestanding [`flite`](https://github.com/accessible-firmware/flite) port (the `flite-freestanding` branch)
* `git clone -b flite-freestanding https://github.com/accessible-firmware/flite`
* point the flite dependency in `hdaudio/Cargo.toml` at the cloned `freestanding-port/flite-freestanding/`
* `cd ihdad`
* `cargo +nightly build --target x86_64-unknown-uefi`
* `cd ..`
* `./qemu.sh`
* once inside QEMU
* `FS0:`
* `.\ihdad.efi`
* type a message at the prompt and press enter to hear it spoken

## Previous Problems

* [X] Could read, but not write values.
  * Fixed by using direct memory access instead of memory-mapped IO.
	* Fixed a second time on AMD Framework boards by soft-resetting the IOMMU; without it, writes still made no sound.
* [X] Could not read updates to RIRBWP address (in QEMU).
  * Fixed in QEMU by adding the: `-device ich9-intel-hda -device hda-duplex`
* [X] Random crashes
  * Fixed by removing `syscall::*` functions and replacing with `std::` functions, where possible.
  * `HashMap -> BTreeMap`: `HashMap`s instantly crash the program; not implemented in UEFI std.
* [X] Leaving the `HdaDevice::configure()` function caused an instant crash.
	* This is confirmed as due to the `drop(...)` implementation of the `path` variable causing the crash. If the `drop()` is done early and explicitly, it crashes early.
	* It is not the `Vec<_>` or `(u8, u16)` drop impls that are causing this.... Is there a custom drop somewhere?
	* The value also appears to change midway through. Reading the value twice shows two different values [(0,0),(0,10)], then [(0,0),(0,0)] unsure what that is. Probably some memory magic I'm unaware of.
	* Unknown how to fix.
	* Fixed in 2026 with LLM assistance to find an anachronistic use of MMIO instead of direct memory adresses.
