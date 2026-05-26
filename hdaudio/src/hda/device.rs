#![allow(dead_code)]

use std::cmp;
use std::cmp::min;
use std::collections::BTreeMap;
use std::fmt::Write;
use std::str;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::io::ErrorKind;
use std::io::Result;
use std::io::Error;
use core::alloc::Layout;
use std::alloc::alloc_zeroed;

//use syscall::error::{Error, ErrorKind::PermissionsDenied, "No access", ErrorKind::Other, "Bad F", ErrorKind::InvalidInput, "input failed to process"};
//use syscall::flag::{SEEK_SET, SEEK_CUR, SEEK_END};
const SEEK_SET: usize = 0;
const SEEK_CUR: usize = 1;
const SEEK_END: usize = 2;
//use syscall::io::{Mmio, Io};
//use syscall::scheme::SchemeBlockMut;

use spin::Mutex;

use crate::hda::io::{Mmio, Io};
use crate::flog;
use super::BufferDescriptorListEntry;
use super::common::*;
use super::StreamDescriptorRegs;
use super::StreamBuffer;
use super::BitsPerSample;
use super::CommandBuffer;
use super::HDANode;
use super::OutputStream;

// GCTL - Global Control
const CRST:   u32 = 1 << 0; // 1 bit
const FNCTRL: u32 = 1 << 1; // 1 bit
const UNSOL:  u32 = 1 << 8; // 1 bit

// CORBCTL
const CMEIE:   u8 = 1 << 0; // 1 bit
const CORBRUN: u8 = 1 << 1; // 1 bit

// CORBSIZE
const CORBSZCAP: (u8,u8) = (4, 4);
const CORBSIZE:  (u8,u8) = (0, 2);

// CORBRP
const CORBRPRST: u16 = 1 << 15;

// RIRBWP
const RIRBWPRST: u16 = 1 << 15;

// RIRBCTL
const RINTCTL:   u8 = 1 << 0; // 1 bit
const RIRBDMAEN: u8 = 1 << 1; // 1 bit

// ICS
const ICB:      u16 = 1 << 0;
const IRV:      u16 = 1 << 1;

// CORB and RIRB offset

const COMMAND_BUFFER_OFFSET: usize = 0x40;

const NUM_SUB_BUFFS: usize = 32;
const SUB_BUFF_SIZE: usize = 2048;

enum Handle {
	Todo,
	Pcmout(usize, usize, usize), // Card, index, block_ptr
	Pcmin(usize, usize, usize),  // Card, index, block_ptr
	StrBuf(Vec<u8>,usize),
}

#[repr(packed)]
#[allow(dead_code)]
struct Regs {
	gcap:       Mmio<u16>,
	vmin:       Mmio<u8>,
	vmaj:       Mmio<u8>,
	outpay:     Mmio<u16>,
	inpay:      Mmio<u16>,
	gctl:       Mmio<u32>,
	wakeen:     Mmio<u16>,
	statests:   Mmio<u16>,
	gsts:       Mmio<u16>,
	rsvd0:      [Mmio<u8>;  6],
	outstrmpay: Mmio<u16>,
	instrmpay:  Mmio<u16>,
	rsvd1:      [Mmio<u8>;  4],
	intctl:     Mmio<u32>,
	intsts:     Mmio<u32>,
	rsvd2:      [Mmio<u8>;  8],
	walclk:     Mmio<u32>,
	rsvd3:      Mmio<u32>,
	ssync:      Mmio<u32>,
	rsvd4:      Mmio<u32>,

	corblbase:  Mmio<u32>,
	corbubase:  Mmio<u32>,
	corbwp:     Mmio<u16>,
	corbrp:     Mmio<u16>,
	corbctl:    Mmio<u8>,
	corbsts:    Mmio<u8>,
	corbsize:   Mmio<u8>,
	rsvd5:      Mmio<u8>,

	rirblbase:  Mmio<u32>,
	rirbubase:  Mmio<u32>,
	rirbwp:     Mmio<u16>,
	rintcnt:    Mmio<u16>,
	rirbctl:    Mmio<u8>,
	rirbsts:    Mmio<u8>,
	rirbsize:   Mmio<u8>,
	rsvd6:      Mmio<u8>,

	icoi:       Mmio<u32>,
	irii:       Mmio<u32>,
	ics:        Mmio<u16>,
	rsvd7:      [Mmio<u8>;  6],

	dplbase:    Mmio<u32>, // 0x70
	dpubase:    Mmio<u32>, // 0x74
}

pub struct IntelHDA {
	vend_prod: u32,

	base: usize,
	regs: &'static mut Regs,

	//corb_rirb_base_phys: usize,

	cmd: CommandBuffer,

	codecs: Vec<CodecAddr>,

	outputs: Vec<WidgetAddr>,
	inputs: Vec<WidgetAddr>,

	widget_map: BTreeMap<WidgetAddr, HDANode>,

	output_pins: Vec<WidgetAddr>,
	input_pins: Vec<WidgetAddr>,

	beep_addr: WidgetAddr,

	buff_desc: &'static mut [BufferDescriptorListEntry; 256],
	buff_desc_phys: usize,   // host (CPU) virtual address of the BDL
	buff_desc_dev: usize,    // device (DMA) address of the BDL (IOMMU-mapped)

	output_streams:         Vec<OutputStream>,

	buffs: Vec<Vec<StreamBuffer>>,

	dma_pos_addr: usize,   // host (CPU) virtual address of DMA position buffer
	dma_pos_dev: usize,    // device (DMA) address of DMA position buffer

	int_counter: usize,
	handles: Mutex<BTreeMap<usize, Handle>>,
	next_id: AtomicUsize,
}

impl IntelHDA {
	pub unsafe fn new(base: usize, vend_prod:u32) -> Result<Self> {
    super::filelog::init();
    flog!("IntelHDA::new(base={:#X}, vend_prod={:#010X})", base, vend_prod);
		let regs = &mut *(base as *mut Regs);

    // BDL must be DMA-readable by the controller. Allocate it through the
    // IOMMU-aware PciIo path when available; host addr for CPU, device addr
    // for SDnBDPL. Falls back to plain alloc (host == device) without IOMMU.
    let (buff_desc_host, buff_desc_device): (usize, usize) = match crate::efidma::alloc_common(1) {
        Some((h, d)) => (h, d),
        None => {
            let buf_layout = Layout::from_size_align(0x1000, 0x1000).unwrap();
            let p = unsafe { alloc_zeroed(buf_layout) } as usize;
            (p, p)
        }
    };
    let buff_desc_virt: usize = buff_desc_host;
		let buff_desc = &mut *(buff_desc_host as *mut [BufferDescriptorListEntry;256]);
    //println!("BUF");

    let cmd_layout = Layout::from_size_align(0x1000, 0x1000).unwrap();
    let cmd_buff_address: *mut u8 = unsafe { alloc_zeroed(cmd_layout) };
    let cmd_buff_virt: usize = cmd_buff_address as usize;
    //println!("CMDBUF");

		//println!("Virt: {:016X}, Phys: {:016X}", buff_desc_virt, buff_desc_phys);
		//println!("Virt: {:016X}, Phys: {:016X}", cmd_buff_virt, cmd_buff_address);

		let mut module = IntelHDA {
			vend_prod: vend_prod,
			base: base,
			regs: regs,

			cmd: CommandBuffer::new(base + COMMAND_BUFFER_OFFSET, cmd_buff_virt, cmd_buff_virt),

			beep_addr: (0,0),

			widget_map: BTreeMap::<WidgetAddr, HDANode>::new(),

			codecs: Vec::<CodecAddr>::new(),

			outputs: Vec::<WidgetAddr>::new(),
			inputs: Vec::<WidgetAddr>::new(),

			output_pins: Vec::<WidgetAddr>::new(),
			input_pins: Vec::<WidgetAddr>::new(),

			buff_desc: buff_desc,
			buff_desc_phys: buff_desc_virt,
			buff_desc_dev: buff_desc_device,

			output_streams: Vec::<OutputStream>::new(),

			buffs: Vec::<Vec<StreamBuffer>>::new(),

			dma_pos_addr: 0,
			dma_pos_dev: 0,

			int_counter: 0,
			handles: Mutex::new(BTreeMap::new()),
			next_id: AtomicUsize::new(0),
		};

		flog!("calling init()");
		module.init();

		module.info();
		flog!("calling enumerate()");
		module.enumerate();

		flog!("IHDA: Initialization finished (configure deferred to caller).");
		super::filelog::flush();
		log::info!("IHDA: Initialization finished.");
		Ok(module)

	}

	pub fn init(&mut self) -> bool {
		self.reset_controller();

		// Use CORB/RIRB for all hardware by default.
		// Immediate Command Interface is intended for BIOS/firmware,
		// not OS drivers, and is not supported by all controllers
		// (especially AMD). Only fall back to immediate for known-broken CORB/RIRB.
		let use_immediate_command_interface = match self.vend_prod {
			0x8086_2668 => true, // ICH6: known broken CORB/RIRB
			_ => false,
		};

		flog!("init(): use_immediate_command_interface={} (vend_prod={:#010X})", use_immediate_command_interface, self.vend_prod);
		self.cmd.init(use_immediate_command_interface);
		self.init_interrupts();

		// Diagnostic: dump GCAP and key registers after init
		let gcap = self.regs.gcap.read();
		flog!("init() post-setup: GCAP={:#06X} (ISS={}, OSS={}, BSS={}, SDO={}, 64bit={})",
			gcap,
			(gcap >> 8) & 0xF,
			(gcap >> 12) & 0xF,
			(gcap >> 3) & 0xF,
			(gcap >> 1) & 0x3,
			gcap & 1);
		flog!("  GCTL={:#010X} STATESTS={:#06X} INTCTL={:#010X}",
			self.regs.gctl.read(), self.regs.statests.read(), self.regs.intctl.read());

		// Send a test NOP-like command: Get Vendor ID from root node (should return non-zero)
		flog!("init() DIAG: Sending test command - Get Vendor/Device ID from codec 0 root");
		let vendor_id = self.cmd.cmd12((0, 0), 0xF00, 0x00);
		flog!("init() DIAG: Vendor/Device ID response = {:#018X} (low32={:#010X})",
			vendor_id, vendor_id as u32);
		if vendor_id == 0 {
			flog!("init() DIAG: WARNING - Vendor ID is zero! CORB/RIRB may not be working.");
			flog!("init() DIAG: Trying a second command with longer delay...");
			super::delay_us(5000);
			let vendor_id2 = self.cmd.cmd12((0, 0), 0xF00, 0x00);
			flog!("init() DIAG: Second Vendor/Device ID response = {:#018X} (low32={:#010X})",
				vendor_id2, vendor_id2 as u32);
			if vendor_id2 == 0 {
				flog!("init() DIAG: Still zero after retry. CORB/RIRB is likely broken.");
				flog!("init() DIAG: Attempting Immediate Command Interface as fallback...");
				self.cmd.set_use_imm_cmds(true);
				let vendor_id3 = self.cmd.cmd12((0, 0), 0xF00, 0x00);
				flog!("init() DIAG: ICS Vendor/Device ID response = {:#018X} (low32={:#010X})",
					vendor_id3, vendor_id3 as u32);
				if vendor_id3 != 0 {
					flog!("init() DIAG: ICS works! Staying with Immediate Command Interface.");
				} else {
					flog!("init() DIAG: ICS also returned zero. Codec may not be responding.");
					// Switch back to CORB/RIRB as the default
					self.cmd.set_use_imm_cmds(false);
				}
			}
		}
		super::filelog::flush();

		true
	}

	pub fn init_interrupts(&mut self) {
		// TODO: provide a function to enable certain interrupts
		// This just enables the first output stream interupt and the global interrupt

		let iss = self.num_input_streams();
		self.regs.intctl.write((1 << 31) | /* (1 << 30) |*/ (1 << iss));
	}

	pub fn irq(&mut self) -> bool {
		self.int_counter += 1;

		self.handle_interrupts()
	}

	pub fn int_count(&self) -> usize {
		self.int_counter
	}

	pub fn read_node(&mut self, addr: WidgetAddr) -> HDANode {
		let mut node = HDANode::new();
		let mut temp:u64;

		node.addr = addr;
		flog!("read_node({:02X}:{:02X})", addr.0, addr.1);

		temp = self.cmd.cmd12( addr, 0xF00, 0x04);
		flog!("  SubnodeInfo(F00:04) raw={:#018X}", temp);

		node.subnode_count = (temp & 0xff) as u16;
		node.subnode_start = ((temp >> 16) & 0xff) as u16;
		flog!("  subnode_count={}, subnode_start={}", node.subnode_count, node.subnode_start);

		if addr.1 == 0 {
			flog!("  root node, returning early");
			return node;
		}
		temp = self.cmd.cmd12(addr, 0xF00, 0x05);
		node.function_group_type = (temp & 0xff) as u8;
		flog!("  FunctionGroupType(F00:05) raw={:#X} => type={:#04X}", temp, node.function_group_type);

		temp = self.cmd.cmd12(addr, 0xF00, 0x09);
		node.capabilities = temp as u32;
		flog!("  Capabilities(F00:09) raw={:#010X} => widget_type bits={:#X}", node.capabilities, (node.capabilities >> 20) & 0xF);

		temp = self.cmd.cmd12(addr, 0xF00, 0x0E);
		node.conn_list_len = (temp & 0xFF) as u8;
		flog!("  ConnListLen(F00:0E) raw={:#X} => len={}, long_form={}", temp, node.conn_list_len & 0x7F, (node.conn_list_len >> 7) & 1);

		node.connections = self.node_get_connection_list(&node);
		flog!("  connections={:X?}", node.connections);

		node.connection_default = self.cmd.cmd12(addr, 0xF01, 0x00) as u8;
		flog!("  ConnectionDefault(F01:00)={}", node.connection_default);

		node.config_default = self.cmd.cmd12(addr, 0xF1C, 0x00) as u32;
		flog!("  ConfigDefault(F1C:00)={:#010X}", node.config_default);

		node
	}

	pub fn node_get_connection_list(&mut self, node: &HDANode) -> Vec<WidgetAddr> {
		let len_field: u8 = (self.cmd.cmd12(node.addr, 0xF00, 0x0E) & 0xFF) as u8;

		// Highest bit is if addresses are represented in longer notation
		// lower 7 is actual count

		let count:u8 = len_field & 0x7F;
		let use_long_addr: bool = (len_field >> 7) & 0x1 == 1;

		let mut current: u8 = 0;

		let mut list = Vec::<WidgetAddr>::new();

		while current < count {

			let response: u32 = (self.cmd.cmd12(node.addr, 0xF02, current) & 0xFFFFFFFF) as u32;

			if use_long_addr {
				for i in 0..2 {
					let addr_field = ((response >> (16 * i)) & 0xFFFF) as u16;
					let addr = addr_field & 0x7FFF;

					if addr == 0 { break; }

					if (addr_field >> 15) & 0x1 == 0x1 {
						for i in list.pop().unwrap().1 .. (addr + 1) {
							list.push((node.addr.0, i));
						}
					} else {
						list.push((node.addr.0, addr));
					}
				}

			} else {
				for i in 0..4 {
					let addr_field = ((response >> (8 * i)) & 0xff) as u16;
					let addr = addr_field & 0x7F;

					if addr == 0 { break; }

					if (addr_field >> 7) & 0x1 == 0x1 {
						for i in list.pop().unwrap().1 .. (addr + 1) {
							list.push((node.addr.0, i));
						}
					} else {
						list.push((node.addr.0, addr));
					}
				}
			}

			current = list.len() as u8;
		}

		list
	}

	pub fn enumerate(&mut self) {
		self.output_pins.clear();
		self.input_pins.clear();
		self.inputs.clear();
		self.outputs.clear();
		self.widget_map.clear();
		self.beep_addr = (0,0);

		let codecs = self.codecs.clone();
		flog!("enumerate(): codecs={:?} (count={})", codecs, codecs.len());
		if codecs.is_empty() {
			flog!("IHDA: No codecs found!");
			println!("IHDA: No codecs found!");
		}
   for &codec in &codecs {
     flog!("--- Enumerating codec {} ---", codec);
     println!("IHDA: Enumerating codec {}", codec);
     let root = self.read_node((codec, 0));
     flog!("  root: {}", root);
     println!("{}", root);

     let root_count = root.subnode_count;
     let root_start = root.subnode_start;
     flog!("  root_count={}, root_start={}", root_count, root_start);

     for i in 0..root_count {
       let fg_nid = root_start + i;
       flog!("  reading FG node ({}, {})", codec, fg_nid);
       let afg = self.read_node((codec, fg_nid));
       flog!("  FG: {}", afg);
       println!("{}", afg);

       // Only process Audio Function Groups (type 0x01)
       if afg.function_group_type != 0x01 {
         flog!("  SKIP FG {}:{:02X} (type=0x{:02X}, not AFG)", codec, fg_nid, afg.function_group_type);
         println!("IHDA: Skip FG {}:{:02X} (type=0x{:02X}, not AFG)",
              codec, fg_nid, afg.function_group_type);
         continue;
       }

       // Power on AFG before enumerating widgets (required by spec;
       // widgets may return all-zeros if AFG is in low-power state)
       flog!("  Powering on AFG ({}, {})", codec, fg_nid);
       self.set_power_state((codec, fg_nid), 0);
       let mut power_ticks: u32 = 0;
       loop {
         let ps = self.cmd.cmd12((codec, fg_nid), 0xF05, 0x00);
         if (ps & 0xF) == 0 {
           flog!("  AFG reached D0 after {} ticks (ps={:#X})", power_ticks, ps);
           break;
         }
         power_ticks += 1;
         if power_ticks > 50000 {
           flog!("  WARNING: AFG did not reach D0 (ps={:#X}) after {} ticks", ps, power_ticks);
           println!("IHDA: Warning: AFG did not reach D0 (ps=0x{:X})", ps);
           break;
         }	

				}

				let afg_count = afg.subnode_count;
       let afg_start = afg.subnode_start;
       flog!("  AFG {}:{:02X} has {} widgets starting at NID {:#04X}", codec, fg_nid, afg_count, afg_start);
       println!("IHDA: AFG {}:{:02X} has {} widgets starting at NID 0x{:02X}",
            codec, fg_nid, afg_count, afg_start);

       for j in 0..afg_count {
         let nid = afg_start + j;
         let mut widget = self.read_node((codec, nid));
         flog!("  Widget {:02X}:{:02X} type={:?} caps={:#010X} config_default={:#010X}",
              codec, nid, widget.widget_type(), widget.capabilities, widget.config_default);
         println!("Widget {:02X}:{:02X} WT: {:?}", codec, nid, widget.widget_type());
         println!("{}", widget);
         widget.is_widget = true;
         match widget.widget_type() {
           HDAWidgetType::AudioOutput => {
             flog!("    => AudioOutput, addr={:?}", widget.addr);
             self.outputs.push(widget.addr);
           },
           HDAWidgetType::AudioInput  => {
             flog!("    => AudioInput, addr={:?}", widget.addr);
             self.inputs.push(widget.addr);
           },
           HDAWidgetType::BeepGenerator => {
             flog!("    => BeepGenerator, addr={:?}", widget.addr);
             self.beep_addr = widget.addr;
           },
           HDAWidgetType::PinComplex => {
             let config = widget.configuration_default();
             let dd = config.default_device();
             let pc = config.port_connectivity();
             let seq = config.sequence();
             let assoc = config.default_association();
             flog!("    => PinComplex {:02X}: device={:?} port={:?} seq={} assoc={} color={:?} loc={:?}/{:?} config={:#010X}",
                  nid, dd, pc, seq, assoc, config.color(), config.gross_location(), config.geometric_location(), widget.config_default);
             flog!("       is_output()={} is_input()={}", config.is_output(), config.is_input());
             println!("  Pin {:02X}: device={:?} port={:?} config=0x{:08X}",
                  nid, config.default_device(), config.port_connectivity(),
                  widget.config_default);
             if config.is_output() {
               flog!("       => ADDED to output_pins");
               self.output_pins.push(widget.addr);
             } else if config.is_input() {
               flog!("       => ADDED to input_pins");
               self.input_pins.push(widget.addr);
             } else {
               flog!("       => NOT output or input (device={:?}), SKIPPED", config.default_device());
             }
           },
           _ => {
             flog!("    => {:?}, ignored", widget.widget_type());
           },
         }

         self.widget_map.insert(widget.addr(), widget);
       }
			}
		}
   flog!("enumerate() DONE: {} audio_outputs, {} audio_inputs, {} output_pins, {} input_pins",
        self.outputs.len(), self.inputs.len(), self.output_pins.len(), self.input_pins.len());
   flog!("  output_pins={:?}", self.output_pins);
   flog!("  input_pins={:?}", self.input_pins);
   flog!("  outputs(DACs)={:?}", self.outputs);
   flog!("  inputs(ADCs)={:?}", self.inputs);
   println!("IHDA: Enumeration complete: {} outputs, {} inputs, {} output pins, {} input pins",
        self.outputs.len(), self.inputs.len(), self.output_pins.len(), self.input_pins.len());
   super::filelog::flush();
	}

	pub fn find_best_output_pin(&mut self) -> Option<WidgetAddr>{
		flog!("find_best_output_pin(): output_pins={:?} (count={})", self.output_pins, self.output_pins.len());
		print!("FBOP...");
		let outs = &self.output_pins;
		if outs.len() == 0 {
			flog!("  => None (no output pins at all!)");
			println!("None!");
			None
		} else if outs.len() == 1 {
			flog!("  => only one pin: {:?}", outs[0]);
			println!("One({:?})", outs[0]);
			Some(outs[0])
		} else {
			flog!("  multiple output pins, searching by priority...");
			// Dump all pin info for debugging
			for &out in outs {
				if let Some(widget) = self.widget_map.get(&out) {
					let cd = widget.configuration_default();
					flog!("  pin {:?}: device={:?} port={:?} seq={} assoc={} config={:#010X}",
						out, cd.default_device(), cd.port_connectivity(), cd.sequence(), cd.default_association(), widget.config_default);
				}
			}

			//TODO: change output based on "unsolicited response" interrupts
			// Check for devices in this order: Headphone, Speaker, Line Out
			for supported_device in &[
				DefaultDevice::HPOut,
				DefaultDevice::Speaker,
				DefaultDevice::LineOut,
				DefaultDevice::AUX,
				DefaultDevice::DigitalOtherOut,
			] {
				flog!("  pass1: looking for {:?} with seq==0", supported_device);
				for &out in outs {
					let widget = self.widget_map.get(&out).unwrap();
					let cd = widget.configuration_default();
					flog!("    checking {:?}: seq={} device={:?} (want {:?})",
						out, cd.sequence(), cd.default_device(), supported_device);
					print!("look at...{out:?}...");
					print!("Try read!");
					print!("seq={},default={:?},supported={:?}", cd.sequence(), cd.default_device(), supported_device);
					if cd.sequence() == 0 && &cd.default_device() == supported_device {
						flog!("    seq=0, device matches!");
						print!("...seq=0,default=supported");
						// Check for jack detect bit
						let pin_caps = self.cmd.cmd12(widget.addr, 0xF00, 0x0C);
						flog!("    pin_caps={:#010X} (jack_detect={})", pin_caps, (pin_caps >> 2) & 1);
						print!("...PC={pin_caps:x}");
						if pin_caps & (1 << 2) != 0 {
							// Check for presence
							let pin_sense = self.cmd.cmd12(widget.addr, 0xF09, 0);
							flog!("    pin_sense={:#010X} (presence={})", pin_sense, (pin_sense >> 31) & 1);
						print!("...PS={pin_sense:x}");
							// Ignore "not plugged in"; send it?
							// No! This causes instant-crash on FW13-AMD... but unsure why...
							if pin_sense & (1 << 31) == 0 {
								flog!("    SKIPPED: nothing plugged in (presence bit=0)");
								// Skip if nothing is plugged in
								continue;
							}
						}
						flog!("  => FOUND: {:?}", out);
						println!("Found: {out:?}");
						return Some(out);
					}
				}
			}

			flog!("  pass1 found nothing, trying fallback (no seq restriction)...");
			// Fallback: try by device priority without sequence restriction.
			// Speaker before HPOut: if we're in fallback, the speaker is more
			// likely to work than an unplugged headphone jack.
			for supported_device in &[
				DefaultDevice::Speaker,
				DefaultDevice::LineOut,
				DefaultDevice::HPOut,
				DefaultDevice::AUX,
				DefaultDevice::DigitalOtherOut,
			] {
				flog!("  pass2: looking for {:?}", supported_device);
				for &out in outs {
					let widget = self.widget_map.get(&out).unwrap();
					let cd = widget.configuration_default();
					flog!("    checking {:?}: device={:?}", out, cd.default_device());
					if &cd.default_device() == supported_device {
						// For HPOut in fallback, still check jack detect
						if *supported_device == DefaultDevice::HPOut {
							let pin_caps = self.cmd.cmd12(widget.addr, 0xF00, 0x0C);
							if pin_caps & (1 << 2) != 0 {
								let pin_sense = self.cmd.cmd12(widget.addr, 0xF09, 0);
								flog!("    fallback HPOut jack detect: caps={:#X} sense={:#X}", pin_caps, pin_sense);
								if pin_sense & (1 << 31) == 0 {
									flog!("    SKIPPED: HPOut not plugged in");
									continue;
								}
							}
						}
						flog!("  => FALLBACK match: {:?} ({:?})", out, cd.default_device());
						println!("Fallback match: {out:?} ({:?})", cd.default_device());
						return Some(out);
					}
				}
			}

			flog!("  => LAST RESORT: returning first pin {:?}", outs[0]);
			// Last resort: return first available output pin
			println!("Last resort pin: {:?}", outs[0]);
			Some(outs[0])
		}
	}

	pub fn find_path_to_dac(&self, addr: WidgetAddr) -> Option<Vec<WidgetAddr>>{
        flog!("find_path_to_dac({:?})", addr);
        println!("[find_path_to_dac]: {addr:?}");
		let widget = self.widget_map.get(&addr).unwrap();
		flog!("  widget type={:?} connections={:?} connection_default={}",
			widget.widget_type(), widget.connections, widget.connection_default);
		if widget.widget_type() == HDAWidgetType::AudioOutput {
			flog!("  reached AudioOutput (DAC) at {:?}", addr);
			Some(vec![addr])
		} else {
			let connection = widget.connections.get(widget.connection_default as usize);
			flog!("  following connection[{}] = {:?}", widget.connection_default, connection);
			let connection = connection?;
			let mut path = self.find_path_to_dac(*connection)?;
			path.insert(0, addr);
			flog!("  path so far: {:?}", path);
            println!("PATH: {path:?}");
			Some(path)
		}
	}

	/*
	  Here we update the buffers and split them into 128 byte sub chunks
	  because each BufferDescriptorList needs to be 128 byte aligned,
	  this makes it so each of the streams can have up to 128/16 (8) buffer descriptors
	*/
	/*
	  Vec of a Vec was doing something weird and causing the driver to hang.
	  So now we have a set of variables instead.


	  Fixed?
	*/

	// TODO: this function causes the `path` variable to show [(0.0). (0.0)].
	pub fn update_sound_buffers(&mut self) {
		for i in 0..self.buffs.len(){
			for j in 0.. min(self.buffs[i].len(), 128/16 ) {
				self.buff_desc[i * 128/16 + j].set_address(self.buffs[i][j].phys().try_into().unwrap());
				self.buff_desc[i * 128/16 + j].set_length(self.buffs[i][j].length() as u32);
				self.buff_desc[i * 128/16 + j].set_interrupt_on_complete(true);
			}
		}

		let r = self.get_output_stream_descriptor(0).unwrap();

		self.output_streams.push(OutputStream::new(NUM_SUB_BUFFS, SUB_BUFF_SIZE, r));

		let o = self.output_streams.get_mut(0).unwrap();

		// addr() = host (CPU) address, phys() = device (DMA) address. With the
		// IOMMU-mapped common buffer these differ; the BDL must contain the
		// DEVICE address. The mapped buffer is contiguous in device space.
		flog!("update_sound_buffers: audio buffer host={:#X} device={:#X}", o.addr(), o.phys());

		for i in 0..NUM_SUB_BUFFS {
			let blk_dev = o.phys() + o.block_size() * i;
			self.buff_desc[i].set_address(blk_dev as u64);
			self.buff_desc[i].set_length(o.block_size() as u32);
			self.buff_desc[i].set_interrupt_on_complete(true);
		}

		// Flush the entire BDL from CPU cache to memory so the DMA engine
		// reads correct buffer addresses/lengths instead of stale zeros.
		let bdl_size = NUM_SUB_BUFFS * core::mem::size_of::<BufferDescriptorListEntry>();
		super::cache_flush_region(self.buff_desc_phys, bdl_size);
		flog!("update_sound_buffers(): flushed {} bytes of BDL at {:#X}", bdl_size, self.buff_desc_phys);
	}

	pub fn configure(&mut self) {
		flog!("configure() starting");
		super::filelog::flush();
		let outpin = match self.find_best_output_pin() {
			Some(pin) => pin,
			None => {
				flog!("FATAL: find_best_output_pin() returned None!");
				super::filelog::flush();
				panic!("IHDA: No output pins?!");
			}
		};

		flog!("Best pin: {:01X}:{:02X}", outpin.0, outpin.1);
		println!("Best pin: {:01X}:{:02X}", outpin.0, outpin.1);

		let path = self.find_path_to_dac(outpin).unwrap();
		let _ = outpin;

		let dac = *path.last().unwrap();
		let pin = *path.first().unwrap();

		println!("Path to DAC ({}): {:X?}", line!(), path);

		// Set power state 0 (on) for all widgets in path
		for &addr in &path {
			self.set_power_state(addr, 0);
		}
		println!("Path to DAC ({}): {:X?}", line!(), path);

		// Pin enable (0x80 = headphone amp enable, 0x40 = output enable)
		self.cmd.cmd12(pin, 0x707, 0xC0);
		println!("Path to DAC ({}): {:X?}", line!(), path);

		// EAPD enable
		self.cmd.cmd12(pin, 0x70C, 2);
		println!("Path to DAC ({}): {:X?}", line!(), path);

		// Set DAC stream and channel
		self.set_stream_channel(dac, 1, 0);
		println!("Path to DAC ({}): {:X?}", line!(), path);

		self.update_sound_buffers();
		println!("Path to DAC ({}): {:X?}", line!(), path);

		println!("Supported Formats: {:08X}", self.get_supported_formats((0,0x1)));
		println!("Capabilities: {:08X}", self.get_capabilities(path[0]));

		println!("Path to DAC ({}): {:X?}", line!(), path);
		// Create output stream
		let output = self.get_output_stream_descriptor(0).unwrap();

		// Reset stream before configuration (required for real hardware)
		output.reset();

		output.set_address(self.buff_desc_phys);
		output.set_pcm_format(&super::SR_48, BitsPerSample::Bits16, 2);
		output.set_cyclic_buffer_length((NUM_SUB_BUFFS * SUB_BUFF_SIZE) as u32); // number of bytes
		output.set_stream_number(1);
		output.set_last_valid_index((NUM_SUB_BUFFS - 1) as u16);
		output.set_interrupt_on_completion(true);
		// Set DAC converter format
		self.set_converter_format(dac, &super::SR_48, BitsPerSample::Bits16, 2);

		// Get DAC converter format
		//TODO: should validate?
		self.cmd.cmd12(dac, 0xA00, 0);

		println!("Path to DAC ({}): {:X?}", line!(), path);
		// Unmute and set gain to 0dB for input and output amplifiers on all widgets in path.
		// Amp capabilities register (F00:0D / F00:12) layout per HDA spec Figure 91:
		//   bits 6:0   = Offset (step index that produces 0 dB gain)
		//   bits 14:8  = NumSteps (number of gain steps; 0 = fixed)
		//   bits 22:16 = StepSize (in 0.25 dB units)
		//   bit  31    = Mute Capable
		for &addr in &path {
			// Read widget capabilities
			let caps = self.cmd.cmd12(addr, 0xF00, 0x09);

			let left = true;
			let right = true;
			let index = 0;
			let mute = false;

			// Check for input amp
			if (caps & (1 << 1)) != 0 {
				// Read input amplifier capabilities
				let in_caps = self.cmd.cmd12(addr, 0xF00, 0x0D);
				let in_0db = (in_caps & 0x7f) as u8; // Offset = 0dB reference
				let output = false;
				let input = true;
				self.set_amplifier_gain_mute(addr, output, input, left, right, index, mute, in_0db);
				println!("Set {:X?} input gain to 0x{:X} (0dB)", addr, in_0db);
			}

			// Check for output amp
			if (caps & (1 << 2)) != 0 {
				// Read output amplifier capabilities
				let out_caps = self.cmd.cmd12(addr, 0xF00, 0x12);
				let out_0db = (out_caps & 0x7f) as u8; // Offset = 0dB reference
				let output = true;
				let input = false;
				self.set_amplifier_gain_mute(addr, output, input, left, right, index, mute, out_0db);
				println!("Set {:X?} output gain to 0x{:X} (0dB)", addr, out_0db);
			}
		}
		println!("Path to DAC ({}): {:X?}", line!(), path);
		println!("{:?}", path.as_ptr());
		println!("Try to drop path");
		// TODO: this line causes an instant crash
		drop(path);
		println!("Dropping path did nothing");

		//TODO: implement hda-verb?

		output.run();

		println!("Waiting for output 0 to start running...");
		// Poll with timeout and delay to prevent infinite hang
		for i in 0..10000u32 {
			if output.control() & (1 << 1) != 0 {
				break;
			}
			super::delay_us(10);
			if i == 9999 {
				println!("WARNING: timeout waiting for output stream to start running");
			}
		}

		println!("Output 0 CONTROL {:#X} STATUS {:#X} POS {:#X}", output.control(), output.status(), output.link_position());

		let _ = output;
	}
	/*

	pub fn configure_vbox(&mut self) {

		let outpin = self.find_best_output_pin().expect("IHDA: No output pins?!");

		println!("Best pin: {:01X}:{:02X}", outpin.0, outpin.1);

		let path = self.find_path_to_dac(outpin).unwrap();
		println!("Path to DAC: {:X?}", path);

		// Pin enable
		self.cmd.cmd12((0,0xC), 0x707, 0x40);


		// EAPD enable
		self.cmd.cmd12((0,0xC), 0x70C, 2);

		self.set_stream_channel((0,0x3), 1, 0);

		self.update_sound_buffers();


		println!("Supported Formats: {:08X}", self.get_supported_formats((0,0x1)));
		println!("Capabilities: {:08X}", self.get_capabilities((0,0x1)));

		let output = self.get_output_stream_descriptor(0).unwrap();

		output.set_address(self.buff_desc_phys);

		output.set_pcm_format(&super::SR_48, BitsPerSample::Bits16, 2);
		output.set_cyclic_buffer_length((NUM_SUB_BUFFS * SUB_BUFF_SIZE) as u32);
		output.set_stream_number(1);
		output.set_last_valid_index((NUM_SUB_BUFFS - 1) as u16);
		output.set_interrupt_on_completion(true);


		self.set_power_state((0,0x3), 0); // Power state 0 is fully on
		self.set_converter_format((0,0x3), &super::SR_44_1, BitsPerSample::Bits16, 2);


		self.cmd.cmd12((0,0x3), 0xA00, 0);

		// Unmute and set gain for pin complex and DAC
		self.set_amplifier_gain_mute((0,0x3), true, true, true, true, 0, false, 0x7f);
		self.set_amplifier_gain_mute((0,0xC), true, true, true, true, 0, false, 0x7f);

		output.run();

		self.beep(1);

	}

	*/

	pub fn dump_codec(&self, _codec:u8) -> String {
		let mut string = String::new();

		for (_, widget) in self.widget_map.iter() {
			let _ = writeln!(string, "{}", widget);
		}

		string
	}

	// BEEP!!
	pub fn beep(&mut self, div:u8) {
		let addr = self.beep_addr;
		if addr != (0,0) {
			let _ = self.cmd.cmd12(addr, 0xF0A, div);
		}
	}

	pub fn read_beep(&mut self) -> u8 {
		let addr = self.beep_addr;
		if addr != (0,0) {
			self.cmd.cmd12(addr, 0x70A, 0) as u8
		}else{
			0
		}
	}

	pub fn reset_controller(&mut self) -> bool {
		// Pre-reset quiesce: stop all DMA engines before resetting
		// the controller. If firmware left streams running, resetting
		// while DMA is active can cause the controller to DMA into
		// random/stale addresses, causing hard resets.

		// Stop all input stream DMA engines
		let iss = ((self.regs.gcap.read() >> 8) & 0xF) as usize;
		for i in 0..iss {
			unsafe {
				let sd = &mut *((self.base + 0x80 + i * 0x20) as *mut StreamDescriptorRegs);
				sd.set_control(0);
			}
		}
		// Stop all output stream DMA engines
		let oss = ((self.regs.gcap.read() >> 12) & 0xF) as usize;
		for i in 0..oss {
			unsafe {
				let sd = &mut *((self.base + 0x80 + iss * 0x20 + i * 0x20) as *mut StreamDescriptorRegs);
				sd.set_control(0);
			}
		}

		// Stop CORB and RIRB DMA engines
		self.regs.corbctl.write(0);
		self.regs.rirbctl.write(0);

		// Reset DMA position buffer
		self.regs.dplbase.write(0);
		self.regs.dpubase.write(0);

		super::delay_us(100);

		self.regs.statests.write(0xFFFF);

		// 3.3.7 - Controller Reset
		// Clear CRST bit to enter reset
		self.regs.gctl.writef(CRST, false);

		// Poll until CRST reads 0 (reset entered), with 10us delays
		// Timeout after 10000 iterations (~100ms)
		for _ in 0..10000 {
			if ! self.regs.gctl.readf(CRST) {
				break;
			}
			super::delay_us(10);
		}

		// Mandatory 100us delay after entering reset (HDA spec 4.2.2)
		super::delay_us(100);

		// Set CRST bit to exit reset
		self.regs.gctl.writef(CRST, true);

		// Poll until CRST reads 1 (reset complete), with 10us delays
		// Timeout after 10000 iterations (~100ms)
		for _ in 0..10000 {
			if self.regs.gctl.readf(CRST) {
				break;
			}
			super::delay_us(10);
		}

		// Clear SSYNC - some AMD controllers leave bits set after reset,
		// which prevents stream DMA from running even with RUN=1.
		let ssync_before = self.regs.ssync.read();
		self.regs.ssync.write(0);
		flog!("reset_controller(): SSYNC before={:#010X} after={:#010X}", ssync_before, self.regs.ssync.read());

		// Wait for codec discovery - spec requires minimum 521us,
		// but real hardware (especially AMD) may need up to 2ms
		super::delay_us(2000);

		// Poll STATESTS for codec presence, with delays
		for _ in 0..1000 {
			if self.regs.statests.read() != 0 {
				break;
			}
			super::delay_us(10);
		}

		let statests = self.regs.statests.read();
		flog!("reset_controller(): STATESTS={:#06X}", statests);
		println!("Statests: {:04X}", statests);

		for i in 0..15 {
			if (statests >> i) & 0x1 == 1 {
				flog!("  codec {} detected", i);
				self.codecs.push(i as CodecAddr);
			}
		}
		flog!("  codecs found: {:?}", self.codecs);
		true

	}

	pub fn num_output_streams(&self) -> usize{
		let gcap = self.regs.gcap.read();
		((gcap >> 12) & 0xF) as usize
	}

	pub fn num_input_streams(&self) -> usize{
		let gcap = self.regs.gcap.read();
		((gcap >> 8) & 0xF) as usize
	}

	pub fn num_bidirectional_streams(&self) -> usize{
		let gcap = self.regs.gcap.read();
		((gcap >> 3) & 0xF) as usize
	}

	pub fn num_serial_data_out(&self) -> usize{
		let gcap = self.regs.gcap.read();
		((gcap >> 1) & 0x3) as usize
	}

	pub fn info(&self) {
		//log::info!("Intel HD Audio Version {}.{}", self.regs.vmaj.read(), self.regs.vmin.read());
		//println!("IHDA: Input Streams: {}", self.num_input_streams());
		//println!("IHDA: Output Streams: {}", self.num_output_streams());
		//println!("IHDA: Bidirectional Streams: {}", self.num_bidirectional_streams());
		//println!("IHDA: Serial Data Outputs: {}", self.num_serial_data_out());
		//println!("IHDA: 64-Bit: {}", self.regs.gcap.read() & 1 == 1);
	}

	fn get_input_stream_descriptor(&self, index: usize) -> Option<&'static mut StreamDescriptorRegs> {
		unsafe {
			if index < self.num_input_streams() {
				Some(&mut *((self.base + 0x80 + index * 0x20) as *mut StreamDescriptorRegs))
			}else{
				None
			}
		}
	}

	fn get_output_stream_descriptor(&self, index: usize) -> Option<&'static mut StreamDescriptorRegs> {
		unsafe {
			if index < self.num_output_streams() {
				Some(&mut *((self.base + 0x80 +
							self.num_input_streams() * 0x20 +
							index * 0x20) as *mut StreamDescriptorRegs))
			}else{
				None
			}
		}
	}


	fn get_bidirectional_stream_descriptor(&self, index: usize) -> Option<&'static mut StreamDescriptorRegs> {
		unsafe {
			if index < self.num_bidirectional_streams() {
				Some(&mut *((self.base + 0x80 +
							self.num_input_streams() * 0x20 +
							self.num_output_streams() * 0x20 +
							index * 0x20) as *mut StreamDescriptorRegs))
			}else{
				None
			}
		}
	}

	fn set_dma_position_buff_addr(&mut self, addr: u64) {
		let addr_val = addr & !0x7F;
		self.regs.dplbase.write((addr_val & 0xFFFFFFFF) as u32);
		self.regs.dpubase.write((addr_val >> 32) as u32);
	}


	fn set_stream_channel(&mut self, addr: WidgetAddr, stream: u8, channel:u8) {
		let val = ((stream & 0xF) << 4) | (channel & 0xF);
		self.cmd.cmd12(addr, 0x706, val);
	}

	fn set_power_state(&mut self, addr:WidgetAddr, state:u8) {
		self.cmd.cmd12(addr, 0x705, state & 0xF) as u32;
	}

	fn get_supported_formats(&mut self, addr: WidgetAddr) -> u32 {
		self.cmd.cmd12(addr, 0xF00, 0x0A) as u32
	}

	fn get_capabilities(&mut self, addr: WidgetAddr) -> u32 {
		self.cmd.cmd12(addr, 0xF00, 0x09) as u32
	}

	fn set_converter_format(&mut self, addr:WidgetAddr, sr: &super::SampleRate, bps: BitsPerSample, channels:u8) {
		let fmt = super::format_to_u16(sr, bps, channels);
		self.cmd.cmd4(addr, 0x2, fmt);
	}

	fn set_amplifier_gain_mute(&mut self, addr: WidgetAddr, output:bool, input:bool, left:bool, right:bool, index:u8, mute:bool, gain: u8) {
		let mut payload: u16 = 0;

		if output { payload |= 1 << 15; }
		if input  { payload |= 1 << 14; }
		if left   { payload |= 1 << 13; }
		if right  { payload |= 1 << 12; }
		if mute   { payload |= 1 <<  7; }
		payload |= ((index as u16) & 0x0F) << 8;
		payload |= (gain  as u16) & 0x7F;

		self.cmd.cmd4(addr, 0x3, payload);
	}

	pub fn write_to_output(&mut self, index:u8, buf: &[u8]) -> Result<Option<usize>> {
		let output = self.get_output_stream_descriptor(index as usize).unwrap();
		let os = self.output_streams.get_mut(index as usize).unwrap();

		// Clear any pending completion interrupt status to prevent DMA stall.
		// AMD controllers may gate DMA progress on interrupt acknowledgment.
		output.clear_interrupts();

		let open_block = (output.link_position() as usize) / os.block_size();

		if os.current_block() == (open_block + 3) % NUM_SUB_BUFFS {
			// Block if we already are 3 buffers ahead
			Ok(None)
		} else {
			os.write_block(buf).map(|count| Some(count))
		}
	}

	pub fn output_stream_diag(&mut self, index: u8) -> (u32, u8, u32) {
		let output = self.get_output_stream_descriptor(index as usize).unwrap();
		(output.control(), output.status(), output.link_position())
	}

	/// One-time setup of the output stream DMA infrastructure.
	/// Must be called before any write_to_output() or pin testing.
	pub fn setup_output_stream(&mut self) {
		flog!("setup_output_stream() starting");
		println!("Setting up output stream DMA...");

		self.update_sound_buffers();

		// Set up + enable the DMA Position Buffer. The controller DMA-writes
		// each stream's current link position into this memory buffer. We use
		// it as a probe: if these positions track the SDLPIB register, then
		// device->memory DMA reaches CPU-visible RAM; if they stay zero while
		// SDLPIB advances, the controller's DMA is not hitting our memory
		// (address translation / coherency problem).
		if self.dma_pos_addr == 0 {
			match crate::efidma::alloc_common(1) {
				Some((h, d)) => { self.dma_pos_addr = h; self.dma_pos_dev = d; }
				None => {
					let dp_layout = Layout::from_size_align(0x1000, 0x1000).unwrap();
					let dp = unsafe { alloc_zeroed(dp_layout) } as usize;
					self.dma_pos_addr = dp;
					self.dma_pos_dev = dp;
				}
			}
		}
		super::cache_flush_region(self.dma_pos_addr, 0x1000);
		let dp_dev = self.dma_pos_dev as u64;
		self.regs.dplbase.write(((dp_dev & 0xFFFF_FF80) as u32) | 1); // bit0 = enable
		self.regs.dpubase.write((dp_dev >> 32) as u32);
		flog!("DMA position buffer host={:#X} device={:#X} DPLBASE={:#010X} DPUBASE={:#010X}",
			self.dma_pos_addr, dp_dev, self.regs.dplbase.read(), self.regs.dpubase.read());

		let bdl_dev = self.buff_desc_dev;
		flog!("BDL base host={:#X} device={:#X}", self.buff_desc_phys, bdl_dev);

		let output = self.get_output_stream_descriptor(0).unwrap();

		// Reset stream before configuration (required for real hardware)
		output.reset();

		// SDnBDPL/U is read by the controller's DMA engine -> DEVICE address.
		output.set_address(bdl_dev);
		output.set_pcm_format(&super::SR_48, BitsPerSample::Bits16, 2);
		output.set_cyclic_buffer_length((NUM_SUB_BUFFS * SUB_BUFF_SIZE) as u32);
		output.set_stream_number(1);
		output.set_last_valid_index((NUM_SUB_BUFFS - 1) as u16);
		output.set_interrupt_on_completion(true);
		// Also enable FIFO error and descriptor error interrupts (Linux sets all 3)
		let ctrl = output.control();
		output.set_control(ctrl | (1 << 3) | (1 << 4));

		// Log the stream tag to verify it was set correctly via 32-bit write
		flog!("  stream tag after set_stream_number: {}", output.stream_number());
		flog!("  SD_CTL={:#010X} SD_FORMAT={:#06X} SD_FIFO={} SD_CBL={} SD_LVI={}",
			output.control(), output.pcm_format(), output.fifo_size(),
			output.cyclic_buffer_length(), (NUM_SUB_BUFFS - 1));

		// Log SSYNC before starting stream
		let ssync = self.regs.ssync.read();
		flog!("  SSYNC={:#010X} before stream start", ssync);
		if ssync != 0 {
			flog!("  WARNING: SSYNC non-zero, clearing to unblock DMA");
			self.regs.ssync.write(0);
		}

		output.run();

		// Wait for stream to start running
		for i in 0..10000u32 {
			if output.control() & (1 << 1) != 0 {
				break;
			}
			super::delay_us(10);
			if i == 9999 {
				flog!("WARNING: timeout waiting for output stream to start running");
				println!("WARNING: timeout waiting for output stream to start running");
			}
		}

		flog!("setup_output_stream() done: CTL={:#X} STS={:#X} POS={:#X}",
			output.control(), output.status(), output.link_position());

		// Poll POS multiple times to verify DMA is advancing
		for poll in 0..5u32 {
			super::delay_us(5000);
			let pos = output.link_position();
			let sts = output.status();
			flog!("  DMA poll[{}]: POS={:#X} STS={:#X}", poll, pos, sts);
		}

		println!("Output stream running: CTL={:#X} STS={:#X} POS={:#X}",
			output.control(), output.status(), output.link_position());
		super::filelog::flush();
	}

	/// Fill the entire output-stream ring buffer (stream 0) with a continuous
	/// sine wave that loops seamlessly (an integer number of periods across the
	/// whole buffer), 48 kHz / 16-bit / stereo, then flush it to RAM for DMA.
	///
	/// This bypasses the producer/consumer streaming path entirely so we can
	/// test whether the DAC produces a clean tone at all. Underruns are
	/// impossible: the DMA just loops a fully-populated static buffer.
	pub fn fill_static_tone(&mut self, freq_hz: f64) {
		let os = match self.output_streams.get_mut(0) {
			Some(o) => o,
			None => { flog!("fill_static_tone: no output stream!"); return; }
		};
		let base = os.addr();
		let total_bytes = os.block_size() * os.block_count();
		let total_frames = total_bytes / 4; // 16-bit stereo = 4 bytes/frame
		// Snap to an integer number of periods so the cyclic loop is seamless.
		let periods = ((freq_hz * total_frames as f64) / 48000.0).round().max(1.0);
		let actual = periods * 48000.0 / total_frames as f64;
		for i in 0..total_frames {
			let phase = 2.0 * core::f64::consts::PI * periods * (i as f64) / (total_frames as f64);
			let s = (libm::sin(phase) * 16000.0) as i16;
			let b = s.to_le_bytes();
			unsafe {
				let p = (base + i * 4) as *mut u8;
				*p = b[0];
				*p.add(1) = b[1];
				*p.add(2) = b[0]; // duplicate to right channel
				*p.add(3) = b[1];
			}
		}
		super::cache_flush_region(base, total_bytes);
		flog!("fill_static_tone: req={}Hz actual={}Hz periods={} bytes={} base={:#X}",
			freq_hz as u32, actual as u32, periods as u32, total_bytes, base);
		super::filelog::flush();
	}

	/// Let the (already running) output stream loop the static buffer for
	/// roughly `wraps` passes of the cyclic buffer (~0.34 s each), logging the
	/// DMA link position so we can confirm continuous playback independent of
	/// the acoustic recording. Clock-independent (counts buffer wraps, not time).
	pub fn play_for_wraps(&mut self, wraps: u32) {
		// DMA Position Buffer entry for output stream 0 = global stream index
		// (num_input_streams + 0), each entry is 8 bytes (position in first 4).
		let dpib_index = self.num_input_streams();
		let dpib_off = dpib_index * 8;
		let dma_pos_addr = self.dma_pos_addr;

		let output = self.get_output_stream_descriptor(0).unwrap();
		let mut last = output.link_position();
		let mut wrap_count = 0u32;
		let mut polls = 0u32;
		flog!("play_for_wraps({}) start: POS={:#X} CTL={:#X} STS={:#X} dpib_index={}",
			wraps, last, output.control(), output.status(), dpib_index);
		loop {
			// Clear any stream interrupt status so AMD DMA doesn't stall on BCIS.
			output.clear_interrupts();
			let pos = output.link_position();
			if pos < last {
				wrap_count += 1;
				// Read the controller-written DMA position from memory and
				// compare to the SDLPIB MMIO register (device->memory probe).
				let dpos = if dma_pos_addr != 0 {
					super::cache_flush_region(dma_pos_addr + dpib_off, 8);
					unsafe { core::ptr::read_volatile((dma_pos_addr + dpib_off) as *const u32) }
				} else { 0xDEAD_BEEF };
				flog!("  wrap {} at poll {} (POS {:#X} -> {:#X}) DPIB_mem={:#X} SDLPIB_reg={:#X}",
					wrap_count, polls, last, pos, dpos, pos);
				if wrap_count >= wraps { break; }
			}
			last = pos;
			polls += 1;
			if polls > 5_000_000 { flog!("  play_for_wraps: poll cap hit (no wraps?)"); break; }
			super::delay_us(200);
		}
		let output = self.get_output_stream_descriptor(0).unwrap();
		flog!("play_for_wraps done: wraps={} polls={} final POS={:#X} CTL={:#X} STS={:#X}",
			wrap_count, polls, output.link_position(), output.control(), output.status());
		super::filelog::flush();
	}

	/// Stop output stream 0 DMA so the controller does not keep playing
	/// freed/reused memory after the binary exits back to the UEFI shell.
	pub fn stop_all_output(&mut self) {
		if let Some(o) = self.get_output_stream_descriptor(0) {
			o.stop();
			o.clear_interrupts();
			flog!("stop_all_output: stream 0 stopped, CTL={:#X}", o.control());
		}
		super::filelog::flush();
	}

	/// Configure a specific pin for audio output.
	/// Returns the DAC address if a path to a DAC was found, None otherwise.
	pub fn configure_pin_for_output(&mut self, pin: WidgetAddr) -> Option<WidgetAddr> {
		flog!("configure_pin_for_output(pin={:02X}:{:02X})", pin.0, pin.1);

		let path = match self.find_path_to_dac(pin) {
			Some(p) => p,
			None => {
				flog!("  no path to DAC from {:?}", pin);
				println!("  No path to DAC from {:02X}:{:02X}", pin.0, pin.1);
				return None;
			}
		};

		let dac = *path.last().unwrap();
		flog!("  path={:X?} dac={:02X}:{:02X}", path, dac.0, dac.1);
		println!("  Path: {:X?}, DAC: {:02X}:{:02X}", path, dac.0, dac.1);

		// Power on all widgets in path
		for &addr in &path {
			self.set_power_state(addr, 0);
		}

		// Set connection selects for widgets with multiple inputs (verb 0x701).
		// Linux does this in snd_hda_activate_path(). Without this, audio may
		// be routed to the wrong DAC even though find_path_to_dac found a path.
		for k in 0..path.len() - 1 {
			let addr = path[k];
			let next_addr = path[k + 1];
			// Look up connection list and find index of next widget.
			// Borrow widget_map briefly, then drop borrow before calling cmd.
			let conn_idx = self.widget_map.get(&addr).and_then(|w| {
				if w.connections.len() > 1 {
					w.connections.iter().position(|&c| c == next_addr)
				} else {
					None
				}
			});
			if let Some(idx) = conn_idx {
				self.cmd.cmd12(addr, 0x701, idx as u8);
				flog!("    set connection select on {:02X}:{:02X} to index {} (-> {:02X}:{:02X})",
					addr.0, addr.1, idx, next_addr.0, next_addr.1);
				println!("    conn_sel {:02X}:{:02X} -> idx {} -> {:02X}:{:02X}",
					addr.0, addr.1, idx, next_addr.0, next_addr.1);
			}
		}

		// Pin enable (0x80 = headphone amp enable, 0x40 = output enable)
		self.cmd.cmd12(pin, 0x707, 0xC0);

		// EAPD enable
		self.cmd.cmd12(pin, 0x70C, 2);

		// Set DAC stream and channel (stream 1, channel 0)
		self.set_stream_channel(dac, 1, 0);

		// Set converter format on DAC (must match SD_FORMAT = 0x0011)
		self.set_converter_format(dac, &super::SR_48, BitsPerSample::Bits16, 2);

		// Read back converter format (validation)
		let fmt_readback = self.cmd.cmd12(dac, 0xA00, 0);
		flog!("    DAC format readback={:#X}", fmt_readback);

		// Unmute and set gain to 0dB for all widgets in path.
		// Amp capabilities register layout per HDA spec Figure 91:
		//   bits 6:0   = Offset (0dB reference step)
		//   bits 14:8  = NumSteps
		//   bits 22:16 = StepSize
		//   bit  31    = Mute Capable
		for &addr in &path {
			let caps = self.cmd.cmd12(addr, 0xF00, 0x09);

			let left = true;
			let right = true;
			let index = 0;
			let mute = false;

			// Input amp
			if (caps & (1 << 1)) != 0 {
				let in_caps = self.cmd.cmd12(addr, 0xF00, 0x0D);
				let in_0db = (in_caps & 0x7f) as u8; // Offset = 0dB
				self.set_amplifier_gain_mute(addr, false, true, left, right, index, mute, in_0db);
				flog!("    set {:X?} input gain={:#X} (0dB)", addr, in_0db);
			}

			// Output amp
			if (caps & (1 << 2)) != 0 {
				let out_caps = self.cmd.cmd12(addr, 0xF00, 0x12);
				let out_0db = (out_caps & 0x7f) as u8; // Offset = 0dB
				self.set_amplifier_gain_mute(addr, true, false, left, right, index, mute, out_0db);
				flog!("    set {:X?} output gain={:#X} (0dB)", addr, out_0db);
			}
		}

		flog!("  pin {:02X}:{:02X} configured for output via DAC {:02X}:{:02X}", pin.0, pin.1, dac.0, dac.1);

		// Realtek ALC2xx fixup: the ICS (Immediate Command Interface) returns
		// incorrect connection lists and capabilities for mixer nodes 0x0C/0x0D,
		// reporting them as VendorDefined instead of AudioMixer. These mixers
		// sit in the physical signal path between the pin and DAC:
		//   Pin 0x14 (Speaker) -> Mixer 0x0C -> DAC 0x02
		//   Pin 0x21 (HPOut)   -> Mixer 0x0D -> DAC 0x03
		// Without configuring them, their input amplifiers stay muted after
		// reset, blocking all audio even though the pin and DAC are correct.
		let codec_vid = self.cmd.cmd12((pin.0, 0), 0xF00, 0x00) as u32;
		let vendor = (codec_vid >> 16) & 0xFFFF;
		flog!("  codec VID={:#010X} vendor={:#06X}", codec_vid, vendor);
		if vendor == 0x10EC {
			// Realtek codec - determine output mixer NID from DAC NID
			let mixer_nid: Option<u16> = match dac.1 {
				0x02 => Some(0x0C), // DAC 0x02 routes through Mixer 0x0C
				0x03 => Some(0x0D), // DAC 0x03 routes through Mixer 0x0D
				_ => None,
			};
			if let Some(mix) = mixer_nid {
				let mix_addr: WidgetAddr = (pin.0, mix);
				flog!("  Realtek fixup: configuring mixer {:02X}:{:02X}", mix_addr.0, mix_addr.1);
				println!("  Realtek fixup: configuring mixer {:02X}:{:02X}", mix_addr.0, mix_addr.1);

				// Power on the mixer
				self.set_power_state(mix_addr, 0);

				// Unmute mixer input amp index 0 (the DAC connection).
				// Realtek output mixers have mute-only input amps (offset=0, 0 steps).
				self.set_amplifier_gain_mute(mix_addr, false, true, true, true, 0, false, 0);
				flog!("    unmuted mixer input amp idx 0");

				// Also try unmuting input amp index 1 with mute=true to silence
				// the loopback path (NID 0x0B) and avoid noise.
				self.set_amplifier_gain_mute(mix_addr, false, true, true, true, 1, true, 0);
				flog!("    muted mixer input amp idx 1 (loopback)");

				// Try output amp too (safe no-op if mixer has no output amp)
				self.set_amplifier_gain_mute(mix_addr, true, false, true, true, 0, false, 0x7F);
				flog!("    set mixer output amp (if present)");
			}
		}

		super::filelog::flush();

		// Leak the path Vec to avoid potential drop crash in UEFI
		std::mem::forget(path);

		Some(dac)
	}

	/// Disable output on a specific pin and disconnect its DAC from the stream.
	pub fn disable_pin_output(&mut self, pin: WidgetAddr, dac: WidgetAddr) {
		flog!("disable_pin_output(pin={:02X}:{:02X}, dac={:02X}:{:02X})", pin.0, pin.1, dac.0, dac.1);

		// Disable pin output (clear output enable + headphone amp)
		self.cmd.cmd12(pin, 0x707, 0x00);

		// Disconnect DAC from stream (set stream=0, channel=0)
		self.set_stream_channel(dac, 0, 0);

		// Mute DAC output amplifier
		self.set_amplifier_gain_mute(dac, true, false, true, true, 0, true, 0);

		flog!("  pin disabled, DAC disconnected");
		super::filelog::flush();
	}

	/// Get the number of output pins found during enumeration.
	pub fn output_pin_count(&self) -> usize {
		self.output_pins.len()
	}

	/// Get the address of an output pin by index.
	pub fn output_pin_addr(&self, index: usize) -> WidgetAddr {
		self.output_pins[index]
	}

	/// Get a human-readable description of an output pin.
	pub fn output_pin_info_str(&self, index: usize) -> String {
		let addr = self.output_pins[index];
		if let Some(widget) = self.widget_map.get(&addr) {
			let cd = widget.configuration_default();
			format!(
				"NID {:02X}:{:02X} {:?} {:?} {:?}/{:?} {:?} assoc={} seq={} config={:#010X}",
				addr.0, addr.1,
				cd.default_device(),
				cd.port_connectivity(),
				cd.gross_location(),
				cd.geometric_location(),
				cd.color(),
				cd.default_association(),
				cd.sequence(),
				widget.config_default
			)
		} else {
			format!("NID {:02X}:{:02X} (no widget info)", addr.0, addr.1)
		}
	}

	pub fn handle_interrupts(&mut self) -> bool {
		let intsts = self.regs.intsts.read();
		if ((intsts >> 31) & 1) == 1 {           // Global Interrupt Status
			if ((intsts >> 30) & 1) == 1 {   // Controller Interrupt Status
				self.handle_controller_interrupt();
			}

			let sis = intsts & 0x3FFFFFFF;
			if sis != 0 {
				self.handle_stream_interrupts(sis);
			}
		}
		intsts != 0
	}

	pub fn handle_controller_interrupt(&mut self) {

	}

	pub fn handle_stream_interrupts(&mut self, sis: u32) {
		let iss = self.num_input_streams();
		let oss = self.num_output_streams();
		let bss = self.num_bidirectional_streams();

		for i in 0..iss {
			if ((sis >> i) & 1 ) == 1 {
				let input = self.get_input_stream_descriptor(i).unwrap();
				input.clear_interrupts();
			}
		}

		for i in 0..oss {
			if ((sis >> (i + iss)) & 1 ) == 1 {
				let output = self.get_output_stream_descriptor(i).unwrap();
				output.clear_interrupts();
			}
		}

		for i in 0..bss {
			if ((sis >> (i + iss + oss)) & 1 ) == 1 {
				let bid = self.get_bidirectional_stream_descriptor(i).unwrap();
				bid.clear_interrupts();
			}
		}
	}

	fn validate_path(&mut self, path: &Vec<&str>) -> bool {
		println!("Path: {:?}", path);
		let mut it = path.iter();
		match it.next() {
			Some(card_str) if (*card_str).starts_with("card") => {
				match usize::from_str_radix(&(*card_str)[4..], 10) {
					Ok(card_num) => {
						println!("Card# {}", card_num);
						match it.next() {
							Some(codec_str) if (*codec_str).starts_with("codec#") => {
								match usize::from_str_radix(&(*codec_str)[6..], 10) {
									Ok(_codec_num) => {
										//let id = self.next_id.fetch_add(1, Ordering::SeqCst);
                    					//self.handles.lock().insert(id, Handle::Disk(disk.clone(), 0));
										true

									},
									_ => false,
								}
							},
							Some(pcmout_str) if (*pcmout_str).starts_with("pcmout") => {
								match usize::from_str_radix(&(*pcmout_str)[6..], 10) {
									Ok(pcmout_num) => {
										println!("pcmout {}", pcmout_num);
										true
									},
									_ => false,
								}
							},
							Some(pcmin_str) if (*pcmin_str).starts_with("pcmin") => {
								match usize::from_str_radix(&(*pcmin_str)[6..], 10) {
									Ok(pcmin_num) => {
										println!("pcmin {}", pcmin_num);
										true
									},
									_ => false,
								}
							},
							_ => false,
						}
					},
					_ => false,
				}
			},
			Some(cards_str) if *cards_str == "cards" => {
				true
			},
			_ => false,
		}
	}
}


/*
impl Drop for IntelHDA {
	fn drop(&mut self) {
		log::info!("IHDA: Deallocating IHDA driver.");

	}
}
*/

//impl SchemeBlockMut for IntelHDA {
impl IntelHDA {
	fn open(&mut self, path: &str, _flags: usize, uid: u32, _gid: u32) -> Result<Option<usize>> {
		//let path: Vec<&str>;
		/*
		match str::from_utf8(_path) {
			Ok(p)  => {
					path = p.split("/").collect();
					if !self.validate_path(&path) {
						return Err(Error::new(ErrorKind::InvalidInput, "input failed to process"));

				},
			Err(_) => {return Err(Error::new(ErrorKind::InvalidInput, "input failed to process"));},
		}*/

		// TODO:
		if uid == 0 {
			let handle = match path.trim_matches('/') {
				//TODO: allow multiple codecs
				"codec" => Handle::StrBuf(self.dump_codec(0).into_bytes(), 0),
				_ => Handle::Todo,
			};
			let id = self.next_id.fetch_add(1, Ordering::SeqCst);
			self.handles.lock().insert(id, handle);
			Ok(Some(id))
		} else {
			Err(Error::new(ErrorKind::PermissionDenied, "No access"))
		}
	}

	fn read(&mut self, id: usize, buf: &mut [u8]) -> Result<Option<usize>> {
        let mut handles = self.handles.lock();
        match *handles.get_mut(&id).ok_or(Error::new(ErrorKind::Other, "Unfindable"))? {
			Handle::StrBuf(ref strbuf, ref mut size) => {
				let mut i = 0;
				while i < buf.len() && *size < strbuf.len() {
					buf[i] = strbuf[*size];
					i += 1;
					*size += 1;
				}
				Ok(Some(i))
			},
			_ => Err(Error::new(ErrorKind::Other, "Bad F")),
		}
	}

	fn write(&mut self, id: usize, buf: &[u8]) -> Result<Option<usize>> {
		let index = {
	        let mut handles = self.handles.lock();
	        match handles.get_mut(&id).ok_or(Error::new(ErrorKind::Other, "unfindable"))? {
				Handle::Todo => 0,
				_ => return Err(Error::new(ErrorKind::Other, "Bad F")),
			}
		};

		//log::info!("Int count: {}", self.int_counter);

		self.write_to_output(index, buf)
	}

	fn seek(&mut self, id: usize, pos: isize, whence: usize) -> Result<Option<isize>> {
	    let pos = pos as usize;
		let mut handles = self.handles.lock();
		match *handles.get_mut(&id).ok_or(Error::new(ErrorKind::Other, "unfindable"))? {
			Handle::StrBuf(ref mut strbuf, ref mut size) => {
				let len = strbuf.len() as usize;
				*size = match whence {
					SEEK_SET => cmp::min(len, pos),
					SEEK_CUR => cmp::max(0, cmp::min(len as isize, *size as isize + pos as isize)) as usize,
					SEEK_END => cmp::max(0, cmp::min(len as isize,   len as isize + pos as isize)) as usize,
					_ => return Err(Error::new(ErrorKind::InvalidInput, "input failed to process"))
				};
				Ok(Some(*size as isize))
			},

			_ => Err(Error::new(ErrorKind::InvalidInput, "input failed to process")),
        }
	}

    fn fpath(&mut self, id: usize, buf: &mut [u8]) -> Result<Option<usize>> {
        let mut handles = self.handles.lock();
        let _handle = handles.get_mut(&id).ok_or(Error::new(ErrorKind::Other, "Unfindable"))?;

        let mut i = 0;
        let scheme_path = b"audiohw:";
        while i < buf.len() && i < scheme_path.len() {
            buf[i] = scheme_path[i];
            i += 1;
        }
        Ok(Some(i))
    }

	fn close(&mut self, id: usize) -> Result<Option<usize>> {
		let mut handles = self.handles.lock();
    	handles.remove(&id).ok_or(Error::new(ErrorKind::Other, "unfindable")).and(Ok(Some(0)))
	}
}
