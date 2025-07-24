
use std::{collections::VecDeque, iter::zip, sync::{atomic::{AtomicBool, Ordering}, mpsc::{Receiver, Sender}}, thread::JoinHandle};

use itertools::izip;
use ordered_float::OrderedFloat;

use num::{complex::{Complex32 as C32, ComplexFloat}, Complex, Zero};
use serialport::SerialPort;

static NEED_SKIP: AtomicBool = AtomicBool::new(false);

type Cint64 = num::complex::Complex<i64>;

const BLOCK_SIZE: usize = 96*2;

const SAMPLE_RATE: u32 = 48000;

const WINDOW_SIZE: usize = 96;

pub const CARRIERS: usize = 12;

pub mod hamming;
pub mod codec;

pub mod serial_msg {
    pub const ADC_DATA:    u16 = 0x0001;
    pub const CONS_POINTS: u16 = 0x0002;
}

const CARRIER_FREQS_HZ: [u32; CARRIERS] = [
    7000, 8000, 8500, 9000, 9500, 10000,
    10500, 11000, 11500, 12500, 13000, 14000,
];

#[derive(Clone, Copy)]
pub struct SampleInfo {
    pub index: usize,
    pub adc: i16,
    pub accum: [C32; CARRIERS],
    pub power: f32,
}

pub type ConsPoints = [C32; CARRIERS];

pub struct Demodulator {
    serial_queue: VecDeque<u8>,
    port: Box<dyn SerialPort>,
    adc_tx: Sender<SampleInfo>,
    cons_tx: Sender<ConsPoints>,
    sample_buf: [i32; WINDOW_SIZE],
    accum: [Cint64; CARRIERS],
    sample_head: usize,
    sample_index: usize,
    filt_power: f32,
    packet_cnt: usize,
}

impl Demodulator {
    pub fn recv(port: &str) -> (Receiver<SampleInfo>, Receiver<ConsPoints>) {
        let port =
            serialport::new(port, 115200)
            .timeout(std::time::Duration::MAX)
            .open()
            .expect("unable to open port");

        let (adc_tx, adc_rx) = std::sync::mpsc::channel();
        let (cons_tx, cons_rx) = std::sync::mpsc::channel();

        let mut dem = Demodulator {
            serial_queue: VecDeque::new(),
            port,
            adc_tx,
            cons_tx,
            accum: [Cint64::zero(); CARRIERS],
            sample_buf: [0; WINDOW_SIZE],
            sample_head: 0,
            sample_index: 0,
            filt_power: 10.0e3,
            packet_cnt: 0,
        };

        std::thread::spawn(move || {
            loop {
                dem.poll();
            }
        });

        (adc_rx, cons_rx)
    }

    pub fn refill_queue(&mut self) {
        let mut buf = [0; BLOCK_SIZE];
        let num_read = self.port.read(&mut buf).unwrap();
        self.serial_queue.extend(&buf[0..num_read]);
    }

    pub fn munch_byte(&mut self) -> u8 {
        if self.serial_queue.is_empty() {
            self.refill_queue();
        }
        
        self.serial_queue.pop_front().unwrap()
    }

    pub fn munch_u16(&mut self) -> u16 {
        let lo = self.munch_byte();
        let hi = self.munch_byte();
        u16::from_le_bytes([lo, hi])
    }

    pub fn munch_bytes(&mut self, len: u16) -> Vec<u8> {
        std::iter::from_fn(|| Some(self.munch_byte())).take(len as usize).collect()
    }

    pub fn poll(&mut self) {
        loop {
            if self.munch_byte() == 0xFF {
                break;
            }
        }

        // found 1st header byte, check for second header byte
        if self.munch_byte() != 0x55 {
            return;
        }
        if self.munch_byte() != 0xAA {
            return;
        }
        if self.munch_byte() != 0x00 {
            return;
        }

        let id = self.munch_u16();
        let len = self.munch_u16();
        let dat = self.munch_bytes(len);

        match id {
            serial_msg::ADC_DATA => {
                for num in dat.chunks_exact(2) {
                    let sample = i16::from_le_bytes(num.try_into().unwrap());
                    let info = self.handle_new_sample(sample);
                    self.adc_tx.send(info).unwrap();
                    /*if sample.unsigned_abs() > 1000 {
                        dbg!(idx);
                    }*/
                }
            }
            serial_msg::CONS_POINTS => {
                let mut points = [C32::zero(); CARRIERS];
                for (pt, src) in zip(&mut points, dat.chunks_exact(4)) {
                    pt.re = i16::from_le_bytes([src[0], src[1]]) as f32 / 4096.0;
                    pt.im = i16::from_le_bytes([src[2], src[3]]) as f32 / 4096.0;
                }
                self.cons_tx.send(points).unwrap();
            }
            _ => {
                eprintln!("unknown message {id}")
            }
        }

        self.packet_cnt += 1;

        /*for num in buf.chunks_exact(2) {
        }*/
    }

    pub fn handle_new_sample(&mut self, sample: i16) -> SampleInfo {
        let sample = sample as i32;

        for (acc, freq) in zip(&mut self.accum, CARRIER_FREQS_HZ) {
            let ang = self.sample_head as f64 * (freq as f64 * std::f64::consts::TAU / SAMPLE_RATE as f64);
            let icos = (ang.cos() * (1u64 << 32) as f64) as i64;
            let isin = (ang.sin() * (1u64 << 32) as f64) as i64;

            acc.re += icos * (sample - self.sample_buf[self.sample_head]) as i64;
            acc.im += isin * (sample - self.sample_buf[self.sample_head]) as i64;
        }
        self.sample_buf[self.sample_head] = sample;
        self.sample_head += 1;
        if self.sample_head >= WINDOW_SIZE { self.sample_head = 0; }
        self.sample_index += 1;

        let mut accum = [C32::zero(); CARRIERS];
        let mut power = 0.0;
        for (res, acc) in zip(&mut accum, self.accum) {
            res.re = acc.re as f32 / ((1u64 << 32) as f32) / WINDOW_SIZE as f32;
            res.im = acc.im as f32 / ((1u64 << 32) as f32) / WINDOW_SIZE as f32;
            power += res.re * res.re + res.im * res.im;
        }

        self.filt_power = 0.5 * self.filt_power + 0.5 * power;

        SampleInfo {
            index: self.sample_index,
            adc: sample as i16,
            accum,
            power: self.filt_power,
        }
    }
}

pub fn skip_stream_byte() {
    NEED_SKIP.store(true, Ordering::Relaxed);
}

#[derive(Clone)]
pub struct Decoder {
    pub header_samples: usize,
    pub samples: Vec<(usize, [C32; CARRIERS])>,
    pub mag_adj: [f32; CARRIERS],
    pub phase_adj: [f32; CARRIERS],
    pub iq_adj: [C32; CARRIERS],
    pub is_init: bool,
}

impl Default for Decoder {
    fn default() -> Self {
        Decoder::new()
    }
}

impl Decoder {
    pub fn new() -> Self {
        Decoder {
            header_samples: 0,
            samples: Vec::new(),
            mag_adj: [1.0; CARRIERS],
            phase_adj: [0.0; CARRIERS],
            iq_adj: [C32::new(1.0, 0.0); CARRIERS],
            is_init: false,
        }
    }

    pub fn push(&mut self, (idx, sample): (usize, [C32; CARRIERS]), lock_adj: bool) {
        if self.header_samples == 2 {
            self.samples.push((idx, sample));
        } else if self.header_samples == 1 {
            self.is_init = true;
            // this is the training sample
            /*if !lock_adj {
                for (adj, samp) in zip(&mut self.iq_adj, sample) {
                    *adj = C32::new(4.0, 0.0) / samp;
                }
            }*/
            self.samples.push((idx, sample));
            //self.samples.push((idx, [C32::new(4.0, 0.0); CARRIERS]));
            self.header_samples += 1;
        } else {
            self.header_samples += 1;
        }

        return;

        let mu = 1e-5;
        for c in 0..CARRIERS {
            let samp_adj = sample[c] * self.iq_adj[c];
            self.mag_adj[c] += mu * (3.0 - samp_adj.abs());

            let closest = closest_symbol(samp_adj);
            if matches!((closest.re, closest.im), (1, 0) | (-1, 0) | (0, 1) | (0, -1)) {
                self.phase_adj[c] -= 0.1 * (samp_adj / Complex::new(closest.re as f32, closest.im as f32)).arg();
            }

            self.iq_adj[c] = self.mag_adj[c] * C32::new(self.phase_adj[c].cos(), self.phase_adj[c].sin());
        }

        if self.header_samples == 2 {
            let mut res = [C32::zero(); CARRIERS];
            for (res, samp, adj) in izip!(&mut res, sample, self.iq_adj) {
                *res = samp * adj;
            }
            self.samples.push((idx, res));
        } else if self.header_samples == 1 {
            for c in 0..CARRIERS {
                if !self.is_init {
                    self.mag_adj[c] = 3.0 / sample[c].abs();
                }
                self.phase_adj[c] = -sample[c].arg();
                self.iq_adj[c] = self.mag_adj[c] * C32::new(self.phase_adj[c].cos(), self.phase_adj[c].sin());
            }
            self.is_init = true;
            // this is the training sample
            /*if !lock_adj {
                for (adj, samp) in zip(&mut self.iq_adj, sample) {
                    *adj = C32::new(4.0, 0.0) / samp;
                }
            }*/
            let mut res = [C32::zero(); CARRIERS];
            for (res, samp, adj) in izip!(&mut res, sample, self.iq_adj) {
                *res = samp * adj;
            }
            self.samples.push((idx, res));
            //self.samples.push((idx, [C32::new(4.0, 0.0); CARRIERS]));
            self.header_samples += 1;
        } else {
            self.header_samples += 1;
        }
    }

    pub fn is_empty(&self) -> bool {
        self.header_samples == 0
    }

    pub fn len(&self) -> usize {
        self.header_samples + self.samples.len()
    }

    pub fn clear(&mut self) {
        self.header_samples = 0;
        self.samples.clear();
    }
}

pub fn closest_symbol(cons: C32) -> Complex<i32> {
    SYMBOL_TO_CONSTELLATION
        .iter()
        .copied()
        .min_by_key(|pt| OrderedFloat((C32::new(pt.re as f32, pt.im as f32) - cons).norm_sqr()))
        .unwrap()
}

const SYMBOL_TO_CONSTELLATION_I: [i32; 32] = [ -4,  0, 0, 4,  4, 0,  0, -4, -2, -2, 2,  2,  2, 2, -2, -2, -3,  1, -3, 1, 3, -1,  3, -1, 1, -3, 1,  1, -1, 3, -1, -1 ];
const SYMBOL_TO_CONSTELLATION_Q: [i32; 32] = [  1, -3, 1, 1, -1, 3, -1, -1,  3, -1, 3, -1, -3, 1, -3,  1, -2, -2,  2, 2, 2,  2, -2, -2, 4,  0, 0, -4, -4, 0,  0,  4 ];

pub const SYMBOL_TO_CONSTELLATION: [Complex<i32>; 32] = const {
    let mut i = 0;
    let mut result = [Complex::new(0, 0); 32];
    while i < 32 {
        result[i].re = SYMBOL_TO_CONSTELLATION_I[i];
        result[i].im = SYMBOL_TO_CONSTELLATION_Q[i];
        i += 1;
    }
    result
};

pub static TEST_DATA: [u8; 1509] = *b"\xE3\x05\
Somebody once told me the world is gonna roll me / \
I ain't the sharpest tool in the shed / \
She was looking kind of dumb with her finger and her thumb / \
In the shape of an \"L\" on her forehead / \
Well, the years start comin' and they don't stop comin' / \
Fed to the rules and I hit the ground runnin' / \
Didn't make sense not to live for fun / \
Your brain gets smart but your head gets dumb / \
So much to do, so much to see / \
So what's wrong with taking the backstreets? / \
You'll never know if you don't go / \
You'll never shine if you don't glow / \
Hey now, you're an all star / \
Get your game on, go play / \
Hey now, you're a rock star / \
Get your show on, get paid / \
(And all that glitters is gold) / \
Only shootin' stars break the mold / \
It's a cool place, and they say it gets colder / \
You're bundled up now, wait 'til you get older / \
But the meteor men beg to differ / \
Judging by the hole in the satellite picture / \
The ice we skate is gettin' pretty thin / \
The waters gettin' warm so you might as well swim / \
My world's on fire, how 'bout yours? / \
That's the way I like it and I'll never get bored / \
Hey now, you're an all star / \
Get your game on, go play / \
Hey now, you're a rock star / \
Get your show on, get paid / \
(All that glitters is gold) / \
Only shootin' stars break the mold / \
Go for the moon / \
G-g-g-go for the moon / \
Go for the moon / \
Go-go-go for the moon / \
Hey now, you're an all star / \
Get your game on, go play / \
Hey now, you're a rock star / \
Get your show on, get paid / \
(All that glitters is gold)\0";