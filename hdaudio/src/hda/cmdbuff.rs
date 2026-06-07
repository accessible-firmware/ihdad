//use syscall::io::{Io, Mmio};

use crate::hda::io::{Io, Mmio};
use crate::flog;
use super::common::*;

// CORBCTL
const CMEIE: u8 = 1 << 0; // 1 bit
const CORBRUN: u8 = 1 << 1; // 1 bit

// CORBSIZE
const CORBSZCAP: (u8, u8) = (4, 4);
const CORBSIZE: (u8, u8) = (0, 2);

// CORBRP
const CORBRPRST: u16 = 1 << 15;

// RIRBWP
const RIRBWPRST: u16 = 1 << 15;

// RIRBCTL
const RINTCTL: u8 = 1 << 0; // 1 bit
const RIRBDMAEN: u8 = 1 << 1; // 1 bit

// RIRBSTS (RW1C — write 1 to clear)
const RINTFL: u8 = 1 << 0; // Response Interrupt Flag
const RIRBOIS: u8 = 1 << 2; // Response Overrun Interrupt Status

const CORB_OFFSET: usize = 0x00;
const RIRB_OFFSET: usize = 0x10;
const ICMD_OFFSET: usize = 0x20;

// ICS
const ICB: u16 = 1 << 0;
const IRV: u16 = 1 << 1;

// CORB and RIRB offset

const COMMAND_BUFFER_OFFSET: usize = 0x40;
const CORB_BUFF_MAX_SIZE: usize = 1024;

struct CommandBufferRegs {
    corblbase: Mmio<u32>,
    corbubase: Mmio<u32>,
    corbwp: Mmio<u16>,
    corbrp: Mmio<u16>,
    corbctl: Mmio<u8>,
    corbsts: Mmio<u8>,
    corbsize: Mmio<u8>,
    rsvd5: Mmio<u8>,

    rirblbase: Mmio<u32>,
    rirbubase: Mmio<u32>,
    rirbwp: Mmio<u16>,
    rintcnt: Mmio<u16>,
    rirbctl: Mmio<u8>,
    rirbsts: Mmio<u8>,
    rirbsize: Mmio<u8>,
    rsvd6: Mmio<u8>,
}

struct CorbRegs {
    corblbase: Mmio<u32>,
    corbubase: Mmio<u32>,
    corbwp: Mmio<u16>,
    corbrp: Mmio<u16>,
    corbctl: Mmio<u8>,
    corbsts: Mmio<u8>,
    corbsize: Mmio<u8>,
    rsvd5: Mmio<u8>,
}

struct Corb {
    regs: &'static mut CorbRegs,
    corb_base: *mut u32,
    corb_base_phys: usize,
    corb_count: usize,
}

impl Corb {
    pub fn new(regs_addr: usize, corb_buff_phys: usize, corb_buff_virt: usize) -> Corb {
        crate::ihd_println!("regs addr {:x}", regs_addr);
        unsafe {
            Corb {
                regs: &mut *(regs_addr as *mut CorbRegs),
                corb_base: (corb_buff_virt) as *mut u32,
                corb_base_phys: corb_buff_phys,
                corb_count: 0,
            }
        }
    }
    //Intel 4.4.1.3
    pub fn init(&mut self) {
        flog!("CORB::init() starting");
        self.stop();
        flog!("  CORB stopped (CORBCTL={:#04X})", self.regs.corbctl.read());

        //Determine CORB and RIRB size and allocate buffer

        //3.3.24
        let corbsize_reg = self.regs.corbsize.read();
        let corbszcap = (corbsize_reg >> 4) & 0xF;
        flog!("  CORBSIZE reg={:#04X} szcap={:#X}", corbsize_reg, corbszcap);

        let mut corbsize_bytes: usize = 0;
        let mut corbsize: u8 = 0;

        if (corbszcap & 4) == 4 {
            corbsize = 2;
            corbsize_bytes = 1024;

            self.corb_count = 256;
        } else if (corbszcap & 2) == 2 {
            corbsize = 1;
            corbsize_bytes = 64;

            self.corb_count = 16;
        } else if (corbszcap & 1) == 1 {
            corbsize = 0;
            corbsize_bytes = 8;

            self.corb_count = 2;
        }

        assert!(self.corb_count != 0);
        flog!("  chosen: corbsize={} ({} bytes, {} entries)", corbsize, corbsize_bytes, self.corb_count);

        // Program the selected CORB size into hardware
        self.regs.corbsize.write(corbsize);
        flog!("  CORBSIZE after write={:#04X}", self.regs.corbsize.read());

        let addr = self.corb_base_phys;
        flog!("  CORB buffer phys={:#010X} virt={:p}", addr, self.corb_base);
        self.set_address(addr);
        flog!("  CORBLBASE={:#010X} CORBUBASE={:#010X}",
            self.regs.corblbase.read(), self.regs.corbubase.read());
        self.regs.corbwp.write(0);
        self.reset_read_pointer();
        flog!("  after init: CORBWP={:#06X} CORBRP={:#06X} CORBCTL={:#04X}",
            self.regs.corbwp.read(), self.regs.corbrp.read(), self.regs.corbctl.read());
    }

    pub fn start(&mut self) {
        flog!("CORB::start() CORBCTL before={:#04X}", self.regs.corbctl.read());
        self.regs.corbctl.writef(CORBRUN, true);
        // Verify CORB DMA is running
        super::delay_us(100);
        let ctl = self.regs.corbctl.read();
        flog!("  CORBCTL after={:#04X} (CORBRUN={})", ctl, (ctl >> 1) & 1);
        if ctl & CORBRUN == 0 {
            flog!("  WARNING: CORB DMA did not start!");
        }
    }

    #[inline(never)]
    pub fn stop(&mut self) {
        // Stop CORB DMA with timeout
        for _ in 0..10000 {
            if !self.regs.corbctl.readf(CORBRUN) {
                break;
            }
            self.regs.corbctl.write(0);
            super::delay_us(10);
        }
    }

    pub fn set_address(&mut self, addr: usize) {
        self.regs.corblbase.write((addr & 0xFFFFFFFF) as u32);
        self.regs.corbubase.write(((addr as u64) >> 32) as u32);
    }

    pub fn reset_read_pointer(&mut self) {
        // FreeBSD approach: write the reset bit, then immediately write 0
        // to clear it. The HDA spec says CORBRPRST always reads as 0, but
        // some hardware (e.g., 82801G) is buggy and doesn't clear the bit,
        // which stalls the CORB engine. Don't bother polling.
        self.regs.corbrp.write(CORBRPRST);
        super::delay_us(100);
        self.regs.corbrp.write(0);
    }

    fn send_command(&mut self, cmd: u32) {
        // Decode: codec=(cmd>>28)&0xF, nid=(cmd>>20)&0xFF, verb+data=cmd&0xFFFFF
        let codec = (cmd >> 28) & 0xF;
        let nid = (cmd >> 20) & 0xFF;
        let verb_data = cmd & 0xFFFFF;
        flog!("CORB TX: {:#010X} (codec={} nid={:#04X} verb+data={:#07X})", cmd, codec, nid, verb_data);

        let pre_wp = self.regs.corbwp.read();
        let pre_rp = self.regs.corbrp.read();
        flog!("  CORB pre: WP={:#06X} RP={:#06X} CTL={:#04X}", pre_wp, pre_rp, self.regs.corbctl.read());

        // wait for the commands to finish
        // Timeout after 10000 iterations (~100ms) to prevent infinite hang
        let mut drain_iters = 0u32;
        for i in 0..10000u32 {
            if (self.regs.corbwp.read() & 0xff) == (self.regs.corbrp.read() & 0xff) {
                drain_iters = i;
                break;
            }
            super::delay_us(10);
            if i == 9999 {
                drain_iters = i;
                flog!("CORB: TIMEOUT waiting for commands to finish (wp={:#X} rp={:#X})",
                    self.regs.corbwp.read(), self.regs.corbrp.read());
                crate::ihd_println!("CORB: timeout waiting for commands to finish");
            }
        }
        if drain_iters > 0 {
            flog!("  CORB drain waited {} iters (WP={:#06X} RP={:#06X})",
                drain_iters, self.regs.corbwp.read(), self.regs.corbrp.read());
        }

        let write_pos: usize =
            ((self.regs.corbwp.read() as usize & 0xFF) + 1) % self.corb_count;
        flog!("  CORB writing cmd {:#010X} to entry[{}] (base={:p})", cmd, write_pos, self.corb_base);
        unsafe {
            core::ptr::write_volatile(self.corb_base.offset(write_pos as isize), cmd);
        }

        // Read back the written value to verify the write
        let readback = unsafe { core::ptr::read_volatile(self.corb_base.offset(write_pos as isize)) };
        flog!("  CORB readback entry[{}]={:#010X} (match={})", write_pos, readback, readback == cmd);

        // Flush CORB entry from CPU cache to memory so the DMA engine sees it
        let entry_addr = self.corb_base as usize + write_pos * core::mem::size_of::<u32>();
        super::cache_flush_region(entry_addr, core::mem::size_of::<u32>());

        self.regs.corbwp.write(write_pos as u16);
        flog!("  CORB post: WP={:#06X} RP={:#06X}", self.regs.corbwp.read(), self.regs.corbrp.read());

        crate::ihd_println!("Corb: {:08X}", cmd);
    }
}

struct RirbRegs {
    rirblbase: Mmio<u32>,
    rirbubase: Mmio<u32>,
    rirbwp: Mmio<u16>,
    rintcnt: Mmio<u16>,
    rirbctl: Mmio<u8>,
    rirbsts: Mmio<u8>,
    rirbsize: Mmio<u8>,
    rsvd6: Mmio<u8>,
}

struct Rirb {
    regs: &'static mut RirbRegs,
    rirb_base: *mut u64,
    rirb_base_phys: usize,
    rirb_rp: u16,
    rirb_count: usize,
}

impl Rirb {
    pub fn new(regs_addr: usize, rirb_buff_phys: usize, rirb_buff_virt: usize) -> Rirb {
        unsafe {
            Rirb {
                regs: &mut *(regs_addr as *mut RirbRegs),
                rirb_base: (rirb_buff_virt) as *mut u64,
                rirb_rp: 0,
                rirb_base_phys: rirb_buff_phys,
                rirb_count: 0,
            }
        }
    }
    //Intel 4.4.1.3
    pub fn init(&mut self) {
        flog!("RIRB::init() starting");
        self.stop();
        flog!("  RIRB stopped (RIRBCTL={:#04X})", self.regs.rirbctl.read());

        let rirbsize_reg = self.regs.rirbsize.read();
        let rirbszcap = (rirbsize_reg >> 4) & 0xF;
        flog!("  RIRBSIZE reg={:#04X} szcap={:#X}", rirbsize_reg, rirbszcap);

        let mut rirbsize_bytes: usize = 0;
        let mut rirbsize: u8 = 0;

        if (rirbszcap & 4) == 4 {
            rirbsize = 2;
            rirbsize_bytes = 2048;

            self.rirb_count = 256;
        } else if (rirbszcap & 2) == 2 {
            rirbsize = 1;
            rirbsize_bytes = 128;

            self.rirb_count = 8;
        } else if (rirbszcap & 1) == 1 {
            rirbsize = 0;
            rirbsize_bytes = 16;

            self.rirb_count = 2;
        }

        assert!(self.rirb_count != 0);
        flog!("  chosen: rirbsize={} ({} bytes, {} entries)", rirbsize, rirbsize_bytes, self.rirb_count);

        // Program the selected RIRB size into hardware
        self.regs.rirbsize.write(rirbsize);
        flog!("  RIRBSIZE after write={:#04X}", self.regs.rirbsize.read());

        let addr = self.rirb_base_phys;
        flog!("  RIRB buffer phys={:#010X} virt={:p}", addr, self.rirb_base);
        self.set_address(addr);
        flog!("  RIRBLBASE={:#010X} RIRBUBASE={:#010X}",
            self.regs.rirblbase.read(), self.regs.rirbubase.read());

        self.reset_write_pointer();
        self.rirb_rp = 0;

        self.regs.rintcnt.write(1);
        flog!("  after init: RIRBWP={:#06X} rirb_rp={} RIRBCTL={:#04X} RINTCNT={}",
            self.regs.rirbwp.read(), self.rirb_rp, self.regs.rirbctl.read(), self.regs.rintcnt.read());

        // Dump first few RIRB entries to verify buffer is zeroed
        for i in 0..4u16 {
            let val = unsafe { core::ptr::read_volatile(self.rirb_base.offset(i as isize)) };
            flog!("  RIRB[{}] = {:#018X}", i, val);
        }
    }

    pub fn start(&mut self) {
        flog!("RIRB::start() RIRBCTL before={:#04X}", self.regs.rirbctl.read());
        self.regs.rirbctl.writef(RIRBDMAEN | RINTCTL, true);
        // Verify RIRB DMA is running
        super::delay_us(100);
        let ctl = self.regs.rirbctl.read();
        flog!("  RIRBCTL after={:#04X} (DMAEN={}, RINTCTL={})", ctl, (ctl >> 1) & 1, ctl & 1);
        if ctl & RIRBDMAEN == 0 {
            flog!("  WARNING: RIRB DMA did not start!");
        }
        flog!("  RIRBWP={:#06X} at start time", self.regs.rirbwp.read());
    }

    pub fn stop(&mut self) {
        let mut val = self.regs.rirbctl.read();
        val &= !(RIRBDMAEN);
        self.regs.rirbctl.write(val);
    }

    pub fn set_address(&mut self, addr: usize) {
        self.regs.rirblbase.write((addr & 0xFFFFFFFF) as u32);
        self.regs.rirbubase.write(((addr as u64) >> 32) as u32);
    }

    pub fn reset_write_pointer(&mut self) {
        self.regs.rirbwp.writef(RIRBWPRST, true);
    }

    fn read_response(&mut self) -> u64 {
        let entry_rirbwp = self.regs.rirbwp.read();
        flog!("RIRB read_response(): entry RIRBWP={:#06X} rirb_rp={} rirb_count={}",
            entry_rirbwp, self.rirb_rp, self.rirb_count);

        // wait for response
        // Timeout after 10000 iterations (~100ms) to prevent infinite hang
        let mut poll_iters = 0u32;
        let mut timed_out = false;
        for i in 0..10000u32 {
            if (self.regs.rirbwp.read() & 0xff) != (self.rirb_rp & 0xff) {
                poll_iters = i;
                break;
            }
            super::delay_us(10);
            if i == 9999 {
                poll_iters = i;
                timed_out = true;
                flog!("RIRB: TIMEOUT waiting for response (wp={:#X} rp={:#X})",
                    self.regs.rirbwp.read(), self.rirb_rp);
                crate::ihd_println!("RIRB: timeout waiting for response");
            }
        }
        let exit_rirbwp = self.regs.rirbwp.read();
        flog!("  RIRB poll done: {} iters, timed_out={}, RIRBWP now={:#06X}",
            poll_iters, timed_out, exit_rirbwp);

        let read_pos: u16 = (self.rirb_rp + 1) % self.rirb_count as u16;
        flog!("  reading RIRB[{}] (base={:p}, byte_offset={})", read_pos, self.rirb_base, (read_pos as usize) * 8);

        // Flush RIRB entry from CPU cache so we read DMA-written data from memory
        let entry_addr = self.rirb_base as usize + (read_pos as usize) * core::mem::size_of::<u64>();
        super::cache_flush_region(entry_addr, core::mem::size_of::<u64>());

        let res: u64;
        unsafe {
            res = core::ptr::read_volatile(self.rirb_base.offset(read_pos as isize));
        }

        // Also dump a few surrounding entries for context
        flog!("  RIRB buffer dump around read_pos={}:", read_pos);
        for i in 0..core::cmp::min(4u16, self.rirb_count as u16) {
            let val = unsafe { core::ptr::read_volatile(self.rirb_base.offset(i as isize)) };
            let marker = if i == read_pos { " <-- READ" } else { "" };
            flog!("    RIRB[{}] = {:#018X}{}", i, val, marker);
        }

        self.rirb_rp = read_pos;
        // Low 32 bits = response, high 32 bits = response extended (solicited/unsolicited + codec addr)
        flog!("RIRB RX: {:#018X} (resp={:#010X} ext={:#010X})", res, res & 0xFFFFFFFF, (res >> 32) & 0xFFFFFFFF);

        // Check for suspicious all-zeros response
        if res == 0 {
            flog!("  WARNING: RIRB response is all zeros! RIRBSTS={:#04X} RIRBCTL={:#04X}",
                self.regs.rirbsts.read(), self.regs.rirbctl.read());
        }

        // Acknowledge RINTFL (and any RIRBOIS) so the controller will
        // process the next CORB command. RIRBSTS is RW1C: write a 1 to
        // the bits we want to clear. With RINTCNT=1 the controller
        // raises RINTFL on every response; QEMU enforces the spec
        // strictly and stalls CORB processing until we clear it.
        // (Real silicon is forgiving — but the spec is the spec.)
        self.regs.rirbsts.write(RINTFL | RIRBOIS);

        crate::ihd_println!("Rirb: {:08X}", res);
        res
    }
}

struct ImmediateCommandRegs {
    icoi: Mmio<u32>,
    irii: Mmio<u32>,
    ics: Mmio<u16>,
    rsvd7: [Mmio<u8>; 6],
}

pub struct ImmediateCommand {
    regs: &'static mut ImmediateCommandRegs,
}

impl ImmediateCommand {
    pub fn new(regs_addr: usize) -> ImmediateCommand {
        unsafe {
            ImmediateCommand {
                regs: &mut *(regs_addr as *mut ImmediateCommandRegs),
            }
        }
    }

    pub fn cmd(&mut self, cmd: u32) -> u64 {
        flog!("ICS TX: {:#010X}", cmd);
        // wait for ready
        for i in 0..10000u32 {
            if !self.regs.ics.readf(ICB) {
                break;
            }
            super::delay_us(10);
            if i == 9999 {
                flog!("ICS: TIMEOUT waiting for ready");
                crate::ihd_println!("ICS: timeout waiting for ready");
            }
        }

        // write command
        self.regs.icoi.write(cmd);

        // set ICB bit to send command
        self.regs.ics.writef(ICB, true);

        // wait for IRV bit to be set to indicate a response is latched
        for i in 0..10000u32 {
            if self.regs.ics.readf(IRV) {
                break;
            }
            super::delay_us(10);
            if i == 9999 {
                flog!("ICS: TIMEOUT waiting for IRV");
                crate::ihd_println!("ICS: timeout waiting for IRV");
            }
        }

        // Read the 32-bit Immediate Response register once and zero-extend.
        // The old code read the register twice, duplicating the value in
        // both halves (e.g. 0x10EC029510EC0295 instead of 0x10EC0295).
        let res: u64 = self.regs.irii.read() as u64;

        // clear the bit so we know when the next response comes
        self.regs.ics.writef(IRV, false);

        flog!("ICS RX: {:#018X}", res);
        res
    }
}

pub struct CommandBuffer {
    // regs: &'static mut CommandBufferRegs,
    corb: Corb,
    rirb: Rirb,
    icmd: ImmediateCommand,

    corb_rirb_base_phys: usize,

    use_immediate_cmd: bool,
}

impl CommandBuffer {
    pub fn new(
        regs_addr: usize,
        cmd_buff_frame_phys: usize,
        cmd_buff_frame: usize,
    ) -> CommandBuffer {
        let corb = Corb::new(regs_addr + CORB_OFFSET, cmd_buff_frame_phys, cmd_buff_frame);
        let rirb = Rirb::new(
            regs_addr + RIRB_OFFSET,
            cmd_buff_frame_phys + CORB_BUFF_MAX_SIZE,
            cmd_buff_frame + CORB_BUFF_MAX_SIZE,
        );

        let icmd = ImmediateCommand::new(regs_addr + ICMD_OFFSET);

        let cmdbuff = CommandBuffer {
            corb: corb,
            rirb: rirb,
            icmd: icmd,

            corb_rirb_base_phys: cmd_buff_frame_phys,

            use_immediate_cmd: false,
        };

        cmdbuff
    }

    pub fn init(&mut self, use_imm_cmds: bool) {
        flog!("CommandBuffer::init(use_imm_cmds={})", use_imm_cmds);
        self.corb.init();
        self.rirb.init();
        flog!("CORB/RIRB init done, now starting DMA engines...");
        self.set_use_imm_cmds(use_imm_cmds);
        flog!("CommandBuffer::init() complete");
    }

    pub fn cmd12(&mut self, addr: WidgetAddr, command: u32, data: u8) -> u64 {
        let mut ncmd: u32 = 0;

        ncmd |= (addr.0 as u32 & 0x00F) << 28;
        ncmd |= (addr.1 as u32 & 0x0FF) << 20;
        ncmd |= (command & 0xFFF) << 8;
        ncmd |= (data as u32 & 0x0FF) << 0;
        self.cmd(ncmd)
    }
    pub fn cmd4(&mut self, addr: WidgetAddr, command: u32, data: u16) -> u64 {
        let mut ncmd: u32 = 0;

        ncmd |= (addr.0 as u32 & 0x000F) << 28;
        ncmd |= (addr.1 as u32 & 0x00FF) << 20;
        ncmd |= (command & 0x000F) << 16;
        ncmd |= (data as u32 & 0xFFFF) << 0;
        self.cmd(ncmd)
    }

    pub fn cmd(&mut self, cmd: u32) -> u64 {
        if self.use_immediate_cmd {
            self.cmd_imm(cmd)
        } else {
            self.cmd_buff(cmd)
        }
    }

    pub fn cmd_imm(&mut self, cmd: u32) -> u64 {
        self.icmd.cmd(cmd)
    }

    pub fn cmd_buff(&mut self, cmd: u32) -> u64 {
        self.corb.send_command(cmd);
        self.rirb.read_response()
    }

    pub fn set_use_imm_cmds(&mut self, use_imm: bool) {
        flog!("set_use_imm_cmds({})", use_imm);
        self.use_immediate_cmd = use_imm;

        if self.use_immediate_cmd {
            flog!("  using Immediate Command Interface, stopping CORB/RIRB");
            self.corb.stop();
            self.rirb.stop();
        } else {
            flog!("  using CORB/RIRB, starting DMA engines...");
            self.corb.start();
            self.rirb.start();
            // Small delay after starting DMA engines before sending commands
            super::delay_us(500);
            flog!("  DMA engines started, post-delay CORBCTL={:#04X} RIRBCTL={:#04X}",
                self.corb.regs.corbctl.read(), self.rirb.regs.rirbctl.read());
            flog!("  CORBWP={:#06X} CORBRP={:#06X} RIRBWP={:#06X} rirb_rp={}",
                self.corb.regs.corbwp.read(), self.corb.regs.corbrp.read(),
                self.rirb.regs.rirbwp.read(), self.rirb.rirb_rp);
        }
    }
}
