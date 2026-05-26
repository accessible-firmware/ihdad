//use syscall::PAGE_SIZE;
//use syscall::error::{Error, EIO, Result};
//use syscall::io::{Mmio, Io};
use std::result;
use std::alloc::alloc;
use std::alloc::dealloc;
use std::alloc::Layout;
use std::io::Error;
use std::io::ErrorKind;
use std::cmp::min;
use std::ptr::copy_nonoverlapping;
use std::ptr;
use super::io::Mmio;
use super::io::Io;

pub static PAGE_SIZE: u32 = 4096;

pub enum BaseRate {
	BR44_1,
	BR48,
}


pub struct SampleRate {
	base: BaseRate,
	mult: u16,
	div:  u16,
}

use self::BaseRate::{BR44_1, BR48};

pub const SR_8:      SampleRate = SampleRate {base: BR48  , mult:  1, div:  6};
pub const SR_11_025: SampleRate = SampleRate {base: BR44_1, mult:  1, div:  4};
pub const SR_16:     SampleRate = SampleRate {base: BR48  , mult:  1, div:  3};
pub const SR_22_05:  SampleRate = SampleRate {base: BR44_1, mult:  1, div:  2};
pub const SR_32:     SampleRate = SampleRate {base: BR48  , mult:  2, div:  3};

pub const SR_44_1:   SampleRate = SampleRate {base: BR44_1, mult:  1, div:  1};
pub const SR_48:     SampleRate = SampleRate {base: BR48  , mult:  1, div:  1};
pub const SR_88_1:   SampleRate = SampleRate {base: BR44_1, mult:  2, div:  1};
pub const SR_96:     SampleRate = SampleRate {base: BR48  , mult:  2, div:  1};
pub const SR_176_4:  SampleRate = SampleRate {base: BR44_1, mult:  4, div:  1};
pub const SR_192:    SampleRate = SampleRate {base: BR48  , mult:  4, div:  1};





#[repr(u8)]
pub enum BitsPerSample {
	Bits8  = 0,
	Bits16 = 1,
	Bits20 = 2,
	Bits24 = 3,
	Bits32 = 4,
}



pub fn format_to_u16(sr: &SampleRate, bps: BitsPerSample, channels:u8) -> u16{


	// 3.3.41

	let base:u16 = match sr.base {
		BaseRate::BR44_1 => { 1 << 14},
		BaseRate::BR48   => { 0 },
	};

	let mult = ((sr.mult - 1) & 0x7) << 11;

	let div  = ((sr.div  - 1) & 0x7) <<  8;

	let bits = (bps as u16) << 4;

	let chan = ((channels - 1) & 0xF) as u16;

	let val:u16 = base | mult | div | bits | chan;

	val

}


#[repr(packed)]
pub struct StreamDescriptorRegs {
	ctrl_lo:            Mmio<u16>,
	ctrl_hi:            Mmio<u8>,
	status:             Mmio<u8>,
	link_pos:           Mmio<u32>,
	buff_length:        Mmio<u32>,
	last_valid_index:   Mmio<u16>,
	resv1:              Mmio<u16>,
	fifo_size_:         Mmio<u16>,
	format:             Mmio<u16>,
	resv2:              Mmio<u32>,
	buff_desc_list_lo:  Mmio<u32>,
	buff_desc_list_hi:  Mmio<u32>,

}


impl StreamDescriptorRegs {
	pub fn status(&self) -> u8 {
		self.status.read()
	}

	pub fn set_status(&mut self, status: u8){
		self.status.write(status);
	}

	pub fn control(&self) -> u32 {
		// AMD controllers require 32-bit DWORD access to SD_CTL.
		// Reading as two separate 16-bit + 8-bit accesses can miss updates
		// on some AMD hardware. Read full DWORD, mask off SD_STS (byte 3).
		unsafe {
			let val = core::ptr::read_volatile(
				&self.ctrl_lo as *const Mmio<u16> as *const u32
			);
			val & 0x00FF_FFFF // mask off status byte (bits 31:24)
		}
	}

	pub fn set_control(&mut self, control:u32) {
		// AMD controllers require 32-bit DWORD access to SD_CTL.
		// Writing as split 16-bit + 8-bit can cause the stream tag
		// (bits 23:20) to not take effect, so the DAC never receives data.
		// Write full DWORD with status byte zeroed (writing 0 to W1C bits
		// in SD_STS is a no-op, so this is safe).
		unsafe {
			core::ptr::write_volatile(
				&mut self.ctrl_lo as *mut Mmio<u16> as *mut u32,
				control & 0x00FF_FFFF // zero out status byte
			);
		}
	}

	pub fn pcm_format(&self) -> u16 {
		self.format.read()
	}

	pub fn set_pcm_format(&mut self, sr: &SampleRate, bps: BitsPerSample, channels:u8) {


		// 3.3.41

		let val = format_to_u16(sr,bps,channels);
		self.format.write(val);

	}

	pub fn fifo_size(&self) -> u16 {
		self.fifo_size_.read()
	}

	pub fn set_cyclic_buffer_length(&mut self, length: u32) {
		self.buff_length.write(length);
	}

	pub fn cyclic_buffer_length(&self) -> u32 {
		self.buff_length.read()
	}

	/// Reset stream descriptor per HDA spec section 3.3.35.
	/// Must be called before configuring a stream to ensure it's in a
	/// known state. Firmware may have left streams partially configured.
	pub fn reset(&mut self) {
		// 1. Set SRST bit (bit 0 of control register)
		let ctrl = self.control();
		self.set_control((ctrl & !0x2) | 0x1); // Set SRST, clear RUN

		// 2. Poll until SRST reads back as 1 (acknowledged)
		// Timeout after 10ms (1000 iterations * 10us)
		for _ in 0..1000 {
			if self.control() & 0x1 != 0 {
				break;
			}
			super::delay_us(10);
		}

		// 3. Clear SRST bit
		let ctrl = self.control();
		self.set_control(ctrl & !0x1);

		// 4. Poll until SRST reads back as 0 (reset complete)
		// Timeout after 10ms (1000 iterations * 10us)
		for _ in 0..1000 {
			if self.control() & 0x1 == 0 {
				break;
			}
			super::delay_us(10);
		}
	}

	pub fn run(&mut self) {
		let val = self.control() | (1 << 1);
		self.set_control(val);
	}

	pub fn stop(&mut self) {
		let val = self.control() & !(1 << 1);
		self.set_control(val);
	}

	pub fn stream_number(&self) -> u8 {
		((self.control() >> 20) & 0xF) as u8
	}

	pub fn set_stream_number(&mut self, stream_number: u8) {
		let val = (self.control() & 0x00FFFF ) | (((stream_number & 0xF ) as u32) << 20);
		self.set_control(val);
	}

	pub fn set_address(&mut self, addr: usize) {
		self.buff_desc_list_lo.write( (addr & 0xFFFFFFFF) as u32);
		self.buff_desc_list_hi.write( ( ((addr as u64) >> 32) & 0xFFFFFFFF) as u32);
	}

	pub fn set_last_valid_index(&mut self, index:u16) {
		self.last_valid_index.write(index);
	}

	pub fn link_position(&self) -> u32 {
		self.link_pos.read()
	}

	pub fn set_interrupt_on_completion(&mut self, enable:bool) {
		let mut ctrl = self.control();
		if enable {
			ctrl |= 1 << 2;
		} else {
			ctrl &= !(1 << 2);
		}
		self.set_control(ctrl);
	}

	pub fn buffer_complete(&self) -> bool {
		self.status.readf(1 << 2)
	}

	pub fn clear_interrupts(&mut self) {
		self.status.write(0x7 << 2);
	}

	// get sample size in bytes
	pub fn sample_size(&self) -> usize {
		let format = self.format.read();
		let chan = (format & 0xF) as usize;
		let bits = ((format >> 4) & 0xF) as usize;
		match bits {
			0 => 1 * (chan + 1),
			1 => 2 * (chan + 1),
			_ => 4 * (chan + 1),
		}
	}


}

pub struct OutputStream {
	buff:        StreamBuffer,

	desc_regs:  &'static mut StreamDescriptorRegs,
}


impl OutputStream {
	pub fn new(block_count: usize, block_length: usize, regs: &'static mut StreamDescriptorRegs) -> OutputStream {
		OutputStream {
			buff: StreamBuffer::new(block_length, block_count).unwrap(),

			desc_regs: regs,
		}
	}

	pub fn write_block(&mut self, buf: &[u8]) -> Result<usize, Error> {
		self.buff.write_block(buf)
	}

	pub fn block_size(&self) -> usize {
		self.buff.block_size()
	}

	pub fn block_count(&self) -> usize {
		self.buff.block_count()
	}

	pub fn current_block(&self) -> usize {
		self.buff.current_block()
	}

	pub fn addr(&self) -> usize {
		self.buff.addr()
	}

	pub fn phys(&self) -> usize {
		self.buff.phys()
	}
}




#[repr(packed)]
pub struct BufferDescriptorListEntry {
	addr_low: Mmio<u32>,
	addr_high: Mmio<u32>,
	len:      Mmio<u32>,
	ioc_resv: Mmio<u32>,
}

impl BufferDescriptorListEntry {
	pub fn address(&self) -> u64 {
		(self.addr_low.read() as u64) |
		((self.addr_high.read() as u64) << 32)
	}

	pub fn set_address(&mut self, addr: u64) {
		self.addr_low.write(addr as u32);
		self.addr_high.write((addr >> 32) as u32);
	}

	pub fn length(&self) -> u32 {
		self.len.read()
	}

	pub fn set_length(&mut self, length: u32) {
		self.len.write(length)
	}

	pub fn interrupt_on_completion(&self) -> bool {
		(self.ioc_resv.read() & 0x1) == 0x1
	}

	pub fn set_interrupt_on_complete(&mut self, ioc: bool) {
		self.ioc_resv.writef(1, ioc);
	}
}

pub struct StreamBuffer {
	phys:   usize,
	addr:   usize,

	block_cnt: usize,
	block_len:  usize,

	cur_pos: usize,
}

impl StreamBuffer {
	pub fn new(block_length: usize, block_count: usize) -> result::Result<StreamBuffer, &'static str> {
		// BUG FIX: was block_length + block_count, should be multiplication
		let size = block_length * block_count;
		let pages = (size + 4095) / 4096;

		// Prefer IOMMU-mapped DMA memory via EFI_PCI_IO_PROTOCOL so the
		// controller can actually reach it. addr = host (CPU), phys = device (DMA).
		if let Some((host, device)) = crate::efidma::alloc_common(pages) {
			return Ok(StreamBuffer {
				addr: host,
				phys: device,
				block_len: block_length,
				block_cnt: block_count,
				cur_pos: 0,
			});
		}

		// Fallback (e.g. QEMU without IOMMU): plain allocation, addr == phys.
		let layout = Layout::from_size_align(size, 4096).unwrap();
		let ptr: *mut u8 = unsafe { alloc(layout) };
		unsafe {
			ptr::write_bytes(ptr, 0, block_length * block_count);
		}
		Ok(StreamBuffer {
			addr:   ptr as usize,
			phys: ptr as usize,
			block_len: block_length,
			block_cnt:  block_count,
			cur_pos: 0,
		})
	}

	pub fn length(&self) -> usize {
		self.block_len * self.block_cnt
	}

	/// Host (CPU) virtual address — for writing sample data.
	pub fn addr(&self) -> usize {
		self.addr
	}

	/// Device (DMA) address — for programming into BDL entries / the controller.
	pub fn phys(&self) -> usize {
		self.phys
	}

	pub fn block_size(&self) -> usize {
		self.block_len
	}

	pub fn block_count(&self) -> usize {
		self.block_cnt
	}

	pub fn current_block(&self) -> usize {
		self.cur_pos
	}

	pub fn write_block(&mut self, buf: &[u8]) -> Result<usize, Error> {
		if buf.len() != self.block_size() {
			return Err(Error::new(ErrorKind::Interrupted, "Length != block size"))
		}
		let len = min(self.block_size(), buf.len());


		//log::trace!("Phys: {:X} Virt: {:X} Offset: {:X} Len: {:X}", self.phys(), self.addr(), self.current_block() * self.block_size(), len);
		let dst_addr = self.addr() + self.current_block() * self.block_size();
		unsafe {
			copy_nonoverlapping(buf.as_ptr(), dst_addr as *mut u8, len);
		}

		// Flush written audio data from CPU cache to memory so the DMA engine
		// reads actual audio samples instead of stale zeros.
		super::cache_flush_region(dst_addr, len);

		self.cur_pos += 1;
		self.cur_pos %= self.block_count();

		Ok(len)

	}
}
impl Drop for StreamBuffer {
	fn drop(&mut self) {
		// Intentionally leak. The buffer may be EFI_PCI_IO_PROTOCOL DMA memory
		// (not from the Rust global allocator), so calling dealloc() here would
		// be undefined behavior. This is a short-lived pre-boot program, so the
		// firmware reclaims everything on exit.
		log::debug!("IHDA: StreamBuffer drop (leaked; DMA memory).");
	}
}
