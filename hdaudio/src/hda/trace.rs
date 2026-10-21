//! Lightweight ring recorder for the output write path (diagnostics).
//!
//! `write_to_output` records one compact [`Rec`] per call into a capped
//! ring. [`dump`] formats the ring into the `flog!` buffer plus a summary
//! so a consumer that persists `filelog::contents()` (e.g. the GRUB /
//! PreBoot integrators) can recover the exact per-block pacing trace from
//! real hardware — which position the DMA reported, how full the cyclic
//! ring was, how long between writes, and whether the DMA read pointer
//! overtook the software write cursor (an audible lap).
//!
//! This exists to diagnose the "overlapping speech" replay on AMD silicon
//! where the audible symptom can't be reproduced in QEMU. Cheap enough to
//! leave compiled in; a consumer that doesn't call [`dump`] pays only the
//! per-call push into a bounded VecDeque.

use alloc::collections::VecDeque;
use alloc::string::String;
use alloc::vec::Vec;
use core::fmt::Write as _;
use spin::Mutex;

/// One output-write decision.
#[derive(Clone, Copy)]
pub struct Rec {
    /// Controller WALCLK (24 MHz global clock) sampled at entry — lets the
    /// analysis see the wall-time gap between consecutive writes (a long
    /// gap = the pump/drain was starved for that long).
    pub walclk: u32,
    /// Raw playback position in bytes from the chipset's position source
    /// (SDLPIB on AMD) — exactly the value backpressure keys off.
    pub pos: u32,
    /// DMA read block = pos / block_size (which ring block the controller
    /// is currently playing).
    pub open: u8,
    /// Software write cursor BEFORE this call (next block to be written).
    pub cur: u8,
    /// Ring fill level = blocks of buffered-but-unplayed audio ahead of the
    /// DMA read block, forward around the ring. Small = near underrun/lap;
    /// large (→ ring size) = near full.
    pub fill: u8,
    /// 0 = backpressure blocked (Ok(None)); 1 = wrote a block (Ok(Some));
    /// 2 = write_block errored.
    pub flag: u8,
}

/// Keep the most recent N writes. One short utterance is a few hundred
/// writes; this holds several seconds of navigation so the window around
/// an audible lap survives until the next `dump`.
const CAP: usize = 6000;

static RING: Mutex<VecDeque<Rec>> = Mutex::new(VecDeque::new());

/// Append one write-path record. Called from `write_to_output`.
pub fn record(walclk: u32, pos: u32, open: u8, cur: u8, fill: u8, flag: u8) {
    let mut r = RING.lock();
    if r.len() == CAP {
        r.pop_front();
    }
    r.push_back(Rec { walclk, pos, open, cur, fill, flag });
}

/// Drop everything recorded so far. A consumer calls this at the start of
/// an utterance so a subsequent `dump` shows that utterance alone.
pub fn reset() {
    RING.lock().clear();
}

/// Format the ring into the `flog!` buffer (std / ihdad.efi path). Does
/// NOT clear the ring (call [`reset`] for that).
pub fn dump(label: &str) {
    let s = contents(label);
    if let Ok(txt) = core::str::from_utf8(&s) {
        // One flog! call per line keeps the sequence numbers aligned.
        for line in txt.lines() {
            crate::flog!("{}", line);
        }
    }
}

/// Format the ring into a standalone byte blob — summary line, then one
/// line per record — for a no_std consumer to persist directly (e.g. to a
/// separate ESP file, overwritten per utterance so it stays small). Does
/// NOT clear the ring.
pub fn contents(label: &str) -> Vec<u8> {
    let mut out = String::new();
    let r = RING.lock();
    let n = r.len();
    if n == 0 {
        let _ = write!(out, "TRACE[{}]: (empty)\r\n", label);
        return out.into_bytes();
    }

    // Summary: what matters for the lap diagnosis.
    let mut wrote = 0u32;
    let mut blocked = 0u32;
    let mut errored = 0u32;
    let mut fill_min = u8::MAX;
    let mut fill_max = 0u8;
    let mut pos_backwards = 0u32; // SDLPIB went DOWN between consecutive calls (not a clean wrap)
    let mut lap_events = 0u32; // DMA read block overtook the write cursor
    let mut max_gap = 0u32; // largest WALCLK delta between consecutive calls (~24 ticks/us)
    let mut prev_pos: Option<u32> = None;
    let mut prev_walclk: Option<u32> = None;
    let mut prev_open: Option<u8> = None;
    let mut prev_cur: Option<u8> = None;
    for rec in r.iter() {
        match rec.flag {
            0 => blocked += 1,
            1 => wrote += 1,
            _ => errored += 1,
        }
        if rec.fill < fill_min { fill_min = rec.fill; }
        if rec.fill > fill_max { fill_max = rec.fill; }
        if let Some(p) = prev_pos {
            // Backwards unless it's a legitimate cyclic wrap (pos dropped
            // near the top of the buffer back toward 0).
            if rec.pos < p && (p - rec.pos) < 0x8000 {
                pos_backwards += 1;
            }
        }
        if let (Some(po), Some(pc)) = (prev_open, prev_cur) {
            // A lap: the DMA read block passed the write cursor. Detect it
            // as the open block crossing the (previous) write cursor going
            // forward while the cursor hadn't advanced past it.
            if po != pc && rec.open != po {
                // forward distance the DMA advanced vs where the cursor was
                let crossed = ((pc + 32 - po) % 32) as u32; // blocks of audio that WERE ahead
                let dma_adv = ((rec.open + 32 - po) % 32) as u32;
                if dma_adv >= crossed && crossed != 0 {
                    lap_events += 1;
                }
            }
        }
        if let Some(w) = prev_walclk {
            let gap = rec.walclk.wrapping_sub(w);
            if gap < 0x4000_0000 && gap > max_gap { max_gap = gap; }
        }
        prev_pos = Some(rec.pos);
        prev_walclk = Some(rec.walclk);
        prev_open = Some(rec.open);
        prev_cur = Some(rec.cur);
    }

    let _ = write!(out,
        "TRACE[{}]: n={} wrote={} blocked={} err={} fill=[{}..{}] pos_backwards={} lap_events={} max_gap_walclk={}(~{}us)\r\n",
        label, n, wrote, blocked, errored, fill_min, fill_max,
        pos_backwards, lap_events, max_gap, max_gap / 24
    );

    // Per-record detail. Compact columns: seq | dWALCLK | pos | open cur fill | flag
    let mut prev_w: Option<u32> = None;
    for (i, rec) in r.iter().enumerate() {
        let dgap = match prev_w {
            Some(w) => rec.walclk.wrapping_sub(w),
            None => 0,
        };
        prev_w = Some(rec.walclk);
        let f = match rec.flag { 0 => "blk", 1 => "wr ", _ => "ERR" };
        let _ = write!(out,
            "  t{:04} d{:>7} pos={:#07X} o={:>2} c={:>2} fill={:>2} {}\r\n",
            i, dgap, rec.pos, rec.open, rec.cur, rec.fill, f
        );
    }
    out.into_bytes()
}
