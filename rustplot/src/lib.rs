
use std::{iter::zip, sync::{atomic::{AtomicBool, Ordering}, mpsc::{Receiver, Sender}}, thread::JoinHandle};

use itertools::izip;
use pyo3::prelude::*;

use num::{complex::Complex32 as C32, Complex, Zero};
use serialport::SerialPort;

static NEED_SKIP: AtomicBool = AtomicBool::new(false);

type Cint64 = num::complex::Complex<i64>;

const BLOCK_SIZE: usize = 96;

const SAMPLE_RATE: u32 = 48000;

const WINDOW_SIZE: usize = 96;

pub const CARRIERS: usize = 12;

pub mod hamming;
pub mod codec;

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

pub struct Demodulator {
    port: Box<dyn SerialPort>,
    tx: Sender<SampleInfo>,
    sample_buf: [i32; WINDOW_SIZE],
    accum: [Cint64; CARRIERS],
    sample_head: usize,
    sample_index: usize,
    filt_power: f32,
}

impl Demodulator {
    pub fn recv(port: &str) -> Receiver<SampleInfo> {
        let port =
            serialport::new(port, 115200)
            .timeout(std::time::Duration::MAX)
            .open()
            .expect("unable to open port");

        let (tx, rx) = std::sync::mpsc::channel();

        let mut dem = Demodulator {
            port,
            tx,
            accum: [Cint64::zero(); CARRIERS],
            sample_buf: [0; WINDOW_SIZE],
            sample_head: 0,
            sample_index: 0,
            filt_power: 0.0,
        };

        std::thread::spawn(move || {
            loop {
                dem.poll();
            }
        });

        rx
    }

    pub fn poll(&mut self) {
        if NEED_SKIP.swap(false, Ordering::Relaxed) {
            println!("SKIPPING!");
            let mut tmp = [0; 1];
            self.port.read_exact(&mut tmp).unwrap();
        }

        let mut buf = [0; BLOCK_SIZE];
        self.port.read_exact(&mut buf).unwrap();
        for num in buf.chunks_exact(2) {
            let sample = i16::from_le_bytes(num.try_into().unwrap());
            let info = self.handle_new_sample(sample);
            self.tx.send(info).unwrap();
        }
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

/// Formats the sum of two numbers as string.
#[pyfunction]
fn sum_as_string(a: usize, b: usize) -> PyResult<String> {
    Ok((a + b).to_string())
}

/// A Python module implemented in Rust.
#[pymodule]
fn rustplot(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(sum_as_string, m)?)?;
    Ok(())
}

pub struct Decoder {
    pub header_samples: usize,
    pub samples: Vec<(usize, [C32; CARRIERS])>,
    pub iq_adj: [C32; CARRIERS],
}

impl Decoder {
    pub fn new() -> Self {
        Decoder {
            header_samples: 0,
            samples: Vec::new(),
            iq_adj: [C32::new(1.0, 0.0); CARRIERS],
        }
    }

    pub fn push(&mut self, (idx, sample): (usize, [C32; CARRIERS])) {
        if self.header_samples == 2 {
            let mut res = [C32::zero(); CARRIERS];
            for (res, samp, adj) in izip!(&mut res, sample, self.iq_adj) {
                *res = samp * adj;
            }
            self.samples.push((idx, res));
        } else if self.header_samples == 1 {
            // this is the training sample
            for (adj, samp) in zip(&mut self.iq_adj, sample) {
                *adj = C32::new(4.0, 0.0) / samp;
            }
            self.samples.push((idx, [C32::new(4.0, 0.0); CARRIERS]));
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