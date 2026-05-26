#![allow(dead_code)]
pub mod device;
pub mod stream;
pub mod common;
pub mod node;
pub mod cmdbuff;
pub mod io;
pub mod filelog;

pub use self::stream::*;
pub use self::node::*;

/// Busy-wait delay using TSC (Time Stamp Counter).
///
/// Assumes a minimum TSC frequency of 1 GHz (1000 cycles/us), which is
/// conservative for any modern x86_64 CPU. The actual delay will be at
/// least `us` microseconds.
#[inline(never)]
pub fn delay_us(us: u64) {
    unsafe {
        let start = core::arch::x86_64::_rdtsc();
        // Conservative: assume at least 1 GHz TSC = 1000 cycles per us.
        // Real hardware is typically 2-4 GHz, so delays will be at least
        // as long as requested.
        let target_cycles = us.saturating_mul(1000);
        while core::arch::x86_64::_rdtsc().wrapping_sub(start) < target_cycles {
            core::hint::spin_loop();
        }
    }
}

/// Flush CPU cache lines covering [addr, addr+len) to memory.
/// Ensures DMA coherency on platforms where PCIe snoop isn't working
/// (e.g. AMD Renoir/Cezanne HDA controllers).
#[inline(never)]
pub fn cache_flush_region(addr: usize, len: usize) {
    unsafe {
        let mut offset = 0usize;
        while offset < len {
            core::arch::x86_64::_mm_clflush((addr + offset) as *const u8);
            offset += 64; // x86 cache line size
        }
        core::arch::x86_64::_mm_sfence();
    }
}


/// Walk the current page tables (CR3) to translate a virtual address to its
/// physical address.
///
/// The HDA controller's DMA engines address *physical* memory. If the UEFI
/// environment did not identity-map our allocations (virtual != physical),
/// then programming the controller with virtual addresses sends its DMA to the
/// wrong RAM — reads return stale/foreign data and writes never become visible
/// to the CPU. This lets us hand the controller real physical addresses.
///
/// Assumes the page tables themselves are identity-mapped (true under UEFI
/// boot services), so PTE physical addresses can be dereferenced directly.
#[inline(never)]
pub unsafe fn virt_to_phys(v: usize) -> usize {
    let cr3: usize;
    core::arch::asm!("mov {}, cr3", out(reg) cr3, options(nomem, nostack, preserves_flags));
    let pml4 = (cr3 & 0x000F_FFFF_FFFF_F000) as *const u64;
    let e4 = core::ptr::read_volatile(pml4.add((v >> 39) & 0x1FF));
    if e4 & 1 == 0 { return 0; }
    let pdpt = ((e4 as usize) & 0x000F_FFFF_FFFF_F000) as *const u64;
    let e3 = core::ptr::read_volatile(pdpt.add((v >> 30) & 0x1FF));
    if e3 & 1 == 0 { return 0; }
    if e3 & (1 << 7) != 0 { // 1 GiB page
        return ((e3 as usize) & 0x000F_FFFF_C000_0000) | (v & 0x3FFF_FFFF);
    }
    let pd = ((e3 as usize) & 0x000F_FFFF_FFFF_F000) as *const u64;
    let e2 = core::ptr::read_volatile(pd.add((v >> 21) & 0x1FF));
    if e2 & 1 == 0 { return 0; }
    if e2 & (1 << 7) != 0 { // 2 MiB page
        return ((e2 as usize) & 0x000F_FFFF_FFE0_0000) | (v & 0x1F_FFFF);
    }
    let pt = ((e2 as usize) & 0x000F_FFFF_FFFF_F000) as *const u64;
    let e1 = core::ptr::read_volatile(pt.add((v >> 12) & 0x1FF));
    if e1 & 1 == 0 { return 0; }
    ((e1 as usize) & 0x000F_FFFF_FFFF_F000) | (v & 0xFFF)
}

pub use self::cmdbuff::*;
pub use self::stream::StreamDescriptorRegs;
pub use self::stream::BufferDescriptorListEntry;
pub use self::stream::BitsPerSample;
pub use self::stream::StreamBuffer;
pub use self::device::IntelHDA;
