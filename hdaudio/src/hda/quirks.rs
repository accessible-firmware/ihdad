//! Per-controller workaround ("quirk") profiles, resolved dynamically from
//! the PCI vendor/device id at controller bring-up.
//!
//! HD Audio controllers are nominally spec-compliant, but every real silicon
//! family needs a slightly different bring-up. Rather than scatter
//! `match self.vend_prod { ... }` arms across `device.rs`, all the per-chipset
//! decisions live here as a single [`Quirks`] value that the rest of the
//! driver reads.
//!
//! # Feature flags
//!
//! Each supported chipset family sits behind its own Cargo feature:
//!
//! - `chipset-amd-gen1`      — AMD (Framework 13 AMD, first gen / "Phoenix"
//!   Ryzen 7040). Covers AMD chipset audio (`0x1022`) and the AMD/ATI HDMI
//!   audio function (`0x1002`).
//! - `chipset-intel-12thgen` — Intel 12th-gen PCH (Alder Lake) HD Audio
//!   controller (`0x8086`), and Intel PCH controllers generally.
//!
//! [`Quirks::for_controller`] only matches a family when its feature is
//! compiled in. Enable a *single* feature to ship a driver locked to one
//! board (no foreign-chipset branches); enable *all* of them — the default,
//! and what `--all-features` gives you — to ship ONE generic binary that
//! detects the chipset at runtime and applies the right profile. Anything not
//! matched (feature absent, or genuinely unknown silicon) falls back to
//! [`Quirks::baseline`], the spec-conservative defaults.
//!
//! Deliberately NOT a giant per-device-id table: only the two chipsets we
//! actually ship on are encoded. Vendor-level matching is enough to cover
//! both target Framework boards; device-id refinement is left for the far
//! future (`device_id` is already threaded through for when it's needed).

/// Where the driver reads the output stream's current DMA playback position
/// from. This gates the cyclic-ring producer/consumer backpressure in
/// `write_to_output`, so getting it wrong for a chipset means the ring looks
/// permanently full and audio never plays.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PositionSource {
    /// Read `SDnLPIB` (the stream descriptor Link Position In Buffer MMIO
    /// register) directly, with the DMA Position Buffer left disabled. This
    /// is the AMD path: Linux's AMD workaround (`AZX_DCAPS_AMD_WORKAROUND` →
    /// `POS_FIX_FIFO`) never enables the position buffer because, after a
    /// power-off mid-DMA, the device→memory position write is wedged and the
    /// engine never fills its FIFO. Reading `SDnLPIB` removes that
    /// dependency.
    LinkPosition,

    /// Read the controller-written DMA Position Buffer in host memory
    /// (`DPLBASE`/`DPUBASE`, one 8-byte entry per stream). This is the Intel
    /// path: on Intel PCH (Skylake … Alder Lake / 12th gen) `SDnLPIB` is the
    /// classic unreliable register — it can read stale or stay put — so Linux
    /// defaults Intel to `POS_FIX_POSBUF`. Without this, the 12th-gen
    /// Framework board is silent (the ring never appears to drain) and every
    /// blocking utterance spins to its timeout cap.
    DmaPositionBuffer,
}

/// Output-amplifier gain policy. Internal speakers vary wildly: the AMD
/// Framework speaker buzzes when driven hard (so it wants headroom below
/// 0 dB), while the Intel 12th-gen speaker is barely audible even at 0 dB (so
/// it wants the amp's maximum step). The gain register value is computed from
/// the codec's amp capabilities (`Offset` = 0 dB step, `NumSteps` = max).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum OutputGain {
    /// The amp's maximum step — loudest the codec allows.
    Max,
    /// The 0 dB reference (the amp-caps `Offset` field).
    ZeroDb,
    /// `pct` percent of the step range *below* the 0 dB reference. Useful when
    /// 0 dB is near the top of the range (lots of attenuation, little boost).
    AttenuatePct(u8),
    /// `pct` percent of the amp's max step, measured from 0 (minimum). Robust
    /// regardless of where the 0 dB offset sits — unlike [`AttenuatePct`], it
    /// can't underflow to silence when the offset is low in the range (as on
    /// the IDT codec in the Framework Intel 12th-gen board).
    PctOfMax(u8),
}

/// How the output stream's `RUN` bit is asserted at startup.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StreamStart {
    /// Plain: set `SDnCTL.RUN`. Correct and sufficient on Intel.
    Plain,

    /// AMD: gate the stream via `SSYNC`, set `RUN`, then release the `SSYNC`
    /// gate, retrying until the position actually advances. Works around AMD
    /// controllers that won't begin transferring on a bare `RUN`.
    SsyncGated,
}

/// The full set of per-controller workarounds, resolved once at bring-up and
/// then read by the rest of the driver. `Copy` so it can be stashed in
/// `IntelHDA` and read freely.
#[derive(Clone, Copy, Debug)]
pub struct Quirks {
    /// Run the `GCTL.CRST` enter/exit reset cycle twice, holding the link in
    /// reset ~1 ms each time, to drain a stream DMA engine wedged by a
    /// power-off mid-transfer (seen on Framework 13 AMD). A single spec
    /// cycle is enough on Intel.
    pub double_controller_reset: bool,

    /// Clear `SSYNC` after reset. Some AMD controllers leave `SSYNC` bits set
    /// across reset, which blocks stream DMA even with `RUN=1`.
    pub clear_ssync_after_reset: bool,

    /// Issue a codec Function Reset (verb `0x7FF`) to each Audio Function
    /// Group before enumeration. Recovers an AMD codec converter (e.g.
    /// ALC295) left wedged by a power-off mid-stream. Harmless on a clean
    /// boot but an extra ~10 ms per AFG, so it is off by default.
    pub function_reset_afg: bool,

    /// Where to read the playback position from. See [`PositionSource`].
    pub position_source: PositionSource,

    /// How to start the output stream. See [`StreamStart`].
    pub stream_start: StreamStart,

    /// Output-amplifier gain policy. See [`OutputGain`].
    pub output_gain: OutputGain,

    /// Use the Immediate Command Interface (`ICS`/`ICOI`/`IRII`) instead of
    /// CORB/RIRB. Only ICH6's CORB/RIRB is broken enough to need this.
    pub immediate_command_interface: bool,
}

impl Quirks {
    /// Spec-conservative defaults: a single reset cycle, plain stream start,
    /// `SDnLPIB` position, CORB/RIRB commands, no AMD-specific recovery. Used
    /// when no chipset profile matches (its feature isn't compiled, or the
    /// silicon is genuinely unknown). Not guaranteed to work on every board —
    /// the point of enabling a chipset feature is to get the working quirks.
    pub const fn baseline() -> Self {
        Quirks {
            double_controller_reset: false,
            clear_ssync_after_reset: false,
            function_reset_afg: false,
            position_source: PositionSource::LinkPosition,
            stream_start: StreamStart::Plain,
            output_gain: OutputGain::ZeroDb,
            immediate_command_interface: false,
        }
    }

    /// AMD (Framework 13 AMD, first gen). The robust-reset / `SSYNC`-gated /
    /// `SDnLPIB` profile that the driver was originally hand-tuned to on that
    /// board.
    pub const fn amd_gen1() -> Self {
        Quirks {
            double_controller_reset: true,
            clear_ssync_after_reset: true,
            function_reset_afg: true,
            position_source: PositionSource::LinkPosition,
            stream_start: StreamStart::SsyncGated,
            // AMD Framework speaker buzzes when driven hard → keep headroom.
            output_gain: OutputGain::AttenuatePct(40),
            immediate_command_interface: false,
        }
    }

    /// Intel 12th-gen PCH (Alder Lake). The defining difference from AMD is
    /// [`PositionSource::DmaPositionBuffer`] — Intel `SDnLPIB` is unreliable,
    /// so the ring backpressure must track the DMA Position Buffer instead.
    /// Stream start is plain `RUN`; a single spec reset suffices.
    pub const fn intel_12thgen() -> Self {
        Quirks {
            double_controller_reset: false,
            clear_ssync_after_reset: false,
            function_reset_afg: false,
            position_source: PositionSource::DmaPositionBuffer,
            stream_start: StreamStart::Plain,
            // Intel 12th-gen internal speaker: 0 dB sits low in this codec's
            // range, so "% below 0 dB" underflowed to silence and Max was too
            // loud. Use a fraction of the amp's max instead. Calibration by
            // ear: 50% = still quiet, 100% (Max) = too loud → 75%.
            output_gain: OutputGain::PctOfMax(75),
            immediate_command_interface: false,
        }
    }

    /// ICH6 (`0x8086:0x2668`): known-broken CORB/RIRB → Immediate Command
    /// Interface. Otherwise spec-baseline.
    const fn ich6() -> Self {
        Quirks {
            immediate_command_interface: true,
            ..Quirks::baseline()
        }
    }

    /// Resolve the quirk profile for a controller from its PCI ids.
    ///
    /// `vendor_id` / `device_id` are the raw PCI ids (the driver stores them
    /// as `vend_prod = (vendor << 16) | device`). Matching is gated by Cargo
    /// feature so a single-chipset build drops the other branches; with all
    /// chipset features on (default / `--all-features`) every branch is live
    /// and the right one is picked here at runtime — a generic driver.
    pub fn for_controller(vendor_id: u16, device_id: u16) -> Self {
        // ICH6 is a tiny always-on special case (one device id, command-
        // interface only) — not a shippable "chipset" profile of its own.
        if vendor_id == 0x8086 && device_id == 0x2668 {
            return Quirks::ich6();
        }

        #[cfg(feature = "chipset-amd-gen1")]
        {
            // 0x1022 = AMD chipset audio; 0x1002 = AMD/ATI HDMI audio function.
            if vendor_id == 0x1022 || vendor_id == 0x1002 {
                return Quirks::amd_gen1();
            }
        }

        #[cfg(feature = "chipset-intel-12thgen")]
        {
            if vendor_id == 0x8086 {
                return Quirks::intel_12thgen();
            }
        }

        // Silence unused-variable warnings in builds where no chipset feature
        // consumes the ids.
        let _ = (vendor_id, device_id);
        Quirks::baseline()
    }
}
