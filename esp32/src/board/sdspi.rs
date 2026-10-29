//! SPIKE (not upstream quality): a microSD card in SPI mode on VSPI (SPI3), the pin set the OpenDMD
//! board uses: SCLK 18, MOSI 23, MISO 19, CS 5. Byte-stream model: every transaction is fed to the
//! card byte by byte (full duplex), response bytes come out of a queue. Image from $ESP32SIM_SD_IMG
//! (raw FAT image, copy-on-write in RAM); on drop, if modified and $ESP32SIM_SD_OUT is set, written there.
use esp_soc::board::SpiPins;
use esp_soc::BoardModel;
use std::collections::VecDeque;

pub const PIN_CS: u8 = 5;

fn crc16(d: &[u8]) -> u16 {
    let mut c: u16 = 0;
    for &b in d {
        c ^= (b as u16) << 8;
        for _ in 0..8 { c = if c & 0x8000 != 0 { (c << 1) ^ 0x1021 } else { c << 1 }; }
    }
    c
}

#[derive(PartialEq, Debug)]
enum Wr { None, Token, Data(usize), Crc(usize) }

pub struct SdSpi {
    img: Vec<u8>,
    dirty: bool,
    out: VecDeque<u8>,
    cmd: Vec<u8>,
    idle: bool,
    app: bool,
    stream: Option<u32>, // CMD18: next block to send
    wr: Wr, wr_multi: bool, wr_block: u32, wr_buf: Vec<u8>,
    pub cmds: u64, pub reads: u64, pub writes: u64, pub unknown: Vec<u8>, pub selected_bytes: u64,
    log: bool,
}

impl SdSpi {
    pub fn new() -> Self {
        let img = std::env::var("ESP32SIM_SD_IMG").ok().and_then(|p| std::fs::read(&p).ok()).unwrap_or_default();
        Self { img, dirty: false, out: VecDeque::new(), cmd: vec![], idle: true, app: false, stream: None,
            wr: Wr::None, wr_multi: false, wr_block: 0, wr_buf: vec![], cmds: 0, reads: 0, writes: 0, unknown: vec![], selected_bytes: 0,
            log: std::env::var("ESP32SIM_SD_LOG").is_ok() }
    }
    fn blocks(&self) -> u32 { (self.img.len() / 512) as u32 }
    fn push_block(&mut self, data: &[u8]) {
        self.out.push_back(0xff);
        self.out.push_back(0xfe);
        self.out.extend(data.iter().copied());
        let c = crc16(data);
        self.out.push_back((c >> 8) as u8);
        self.out.push_back(c as u8);
    }
    fn read_block(&mut self, n: u32) {
        let o = n as usize * 512;
        if o + 512 <= self.img.len() {
            let d = self.img[o..o + 512].to_vec();
            self.reads += 1;
            self.push_block(&d);
        } else {
            self.out.push_back(0xff);
            self.out.push_back(0x08); // data error token: out of range
        }
    }
    fn csd(&self) -> [u8; 16] {
        let cs = (self.blocks() / 1024).saturating_sub(1);
        [0x40, 0x0e, 0x00, 0x32, 0x5b, 0x59, 0x00, (cs >> 16) as u8 & 0x3f, (cs >> 8) as u8, cs as u8, 0x7f, 0x80, 0x0a, 0x40, 0x00, 0x01]
    }
    fn cid(&self) -> [u8; 16] {
        [0x03, b'S', b'I', b's', b'p', b'i', b'k', b'e', 0x10, 0x00, 0x00, 0x12, 0x34, 0x01, 0x4a, 0x01]
    }
    fn command(&mut self) {
        let c = std::mem::take(&mut self.cmd);
        let idx = c[0] & 0x3f;
        let arg = u32::from_be_bytes([c[1], c[2], c[3], c[4]]);
        self.cmds += 1;
        let acmd = std::mem::replace(&mut self.app, false);
        if self.log { eprintln!("[sd] {}CMD{idx} arg={arg:#x}", if acmd { "A" } else { "" }); }
        let idle = self.idle as u8;
        if idx == 12 { self.stream = None; self.out.clear(); self.out.extend([0xff, 0x00, 0xff]); return; }
        if self.stream.is_some() && idx != 12 { self.stream = None; self.out.clear(); }
        self.out.push_back(0xff); // NCR
        match (acmd, idx) {
            (_, 0) => { self.idle = true; self.out.push_back(0x01); }
            (_, 8) => { self.out.extend([idle, 0, 0, (arg >> 8) as u8 & 0xf, arg as u8]); }
            (_, 55) => { self.app = true; self.out.push_back(idle); }
            (true, 41) => { self.idle = false; self.out.push_back(0x00); }
            (_, 58) => { self.out.extend([idle, 0xc0, 0xff, 0x80, 0x00]); }
            (true, 51) => { self.out.push_back(0x00); self.push_block(&[0x02, 0x35, 0x80, 0x00, 0, 0, 0, 0]); }
            (true, 13) => { self.out.extend([0x00, 0x00]); self.push_block(&[0u8; 64]); }
            (_, 59) | (_, 16) => self.out.push_back(idle),
            (true, 23) | (_, 23) => self.out.push_back(0x00),
            (_, 13) => self.out.extend([0x00, 0x00]),
            (_, 9) => { self.out.push_back(0x00); let d = self.csd(); self.push_block(&d); }
            (_, 10) => { self.out.push_back(0x00); let d = self.cid(); self.push_block(&d); }
            (_, 17) => { self.out.push_back(0x00); self.read_block(arg); }
            (_, 18) => { self.out.push_back(0x00); self.stream = Some(arg); self.read_block(arg); self.stream = Some(arg + 1); }
            (_, 24) | (_, 25) => { self.out.push_back(0x00); self.wr = Wr::Token; self.wr_multi = idx == 25; self.wr_block = arg; }
            (_, 32) | (_, 33) | (_, 38) => self.out.push_back(0x00),
            _ => { self.unknown.push(idx); self.out.push_back(0x04); }
        }
    }
    fn byte(&mut self, b: u8) -> u8 {
        // output first (response to what came before this byte)
        let o = match self.out.pop_front() {
            Some(v) => v,
            None => {
                if let Some(n) = self.stream { // keep streaming CMD18
                    self.stream = Some(n + 1);
                    self.read_block(n);
                    self.out.pop_front().unwrap_or(0xff)
                } else { 0xff }
            }
        };
        match self.wr {
            Wr::Token => {
                if b == 0xfe || (self.wr_multi && b == 0xfc) { self.wr = Wr::Data(0); self.wr_buf.clear(); }
                else if self.wr_multi && b == 0xfd { self.wr = Wr::None; self.out.extend([0xff, 0x00, 0xff]); }
            }
            Wr::Data(n) => {
                self.wr_buf.push(b);
                self.wr = if n + 1 == 512 { Wr::Crc(0) } else { Wr::Data(n + 1) };
            }
            Wr::Crc(n) => {
                if n == 0 { self.wr = Wr::Crc(1); }
                else {
                    let off = self.wr_block as usize * 512;
                    let ok = off + 512 <= self.img.len();
                    if ok { self.img[off..off + 512].copy_from_slice(&self.wr_buf); self.dirty = true; self.writes += 1; }
                    self.wr_block += 1;
                    self.out.clear();
                    self.out.extend([if ok { 0x05 } else { 0x0d }, 0x00, 0x00, 0xff]);
                    self.wr = if self.wr_multi { Wr::Token } else { Wr::None };
                }
            }
            Wr::None => {
                if self.cmd.is_empty() { if b & 0xc0 == 0x40 { self.cmd.push(b); } }
                else { self.cmd.push(b); if self.cmd.len() == 6 { self.command(); } }
            }
        }
        o
    }
}

impl Drop for SdSpi {
    fn drop(&mut self) {
        if self.dirty { if let Ok(p) = std::env::var("ESP32SIM_SD_OUT") { let _ = std::fs::write(p, &self.img); } }
    }
}

impl BoardModel for SdSpi {
    fn name(&self) -> &'static str { "esp32dev-sdspi" }
    fn uses_spi_pins(&self) -> bool { true }
    fn spi_transfer_pins(&mut self, host: u8, pins: SpiPins, tx: &[u8], rx_len: usize) -> Vec<u8> {
        let n = tx.len().max(rx_len);
        if host != 3 || pins.cs & (1 << PIN_CS) == 0 || self.img.is_empty() {
            if self.log { eprintln!("[sd] transaction ignored: host={host} cs={:#x} tx={} rx={rx_len}", pins.cs, tx.len()); }
            return vec![0xff; rx_len];
        }
        self.selected_bytes += n as u64;
        let mut rx = Vec::with_capacity(n);
        for i in 0..n { rx.push(self.byte(tx.get(i).copied().unwrap_or(0xff))); }
        rx.truncate(rx_len);
        rx
    }
    fn report(&self) -> String {
        format!("[emu] sdspi: {} commands, {} blocks read, {} written, {} bytes clocked, unknown cmds {:?}", self.cmds, self.reads, self.writes, self.selected_bytes, self.unknown)
    }
}
