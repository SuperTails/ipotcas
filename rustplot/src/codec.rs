use bitvec::{field::BitField, order::Lsb0, vec::BitVec, view::BitView};
use num::Complex;

use crate::{hamming, CARRIERS, SYMBOL_TO_CONSTELLATION};

pub fn bytes_to_constellation(bytes: &[u8]) -> Vec<[Complex<i32>; CARRIERS]> {
    // hamming encode
    let mut hamming_encoded = hamming::encode(bytes.view_bits::<Lsb0>());
    while hamming_encoded.len() % CARRIERS != 0 {
        hamming_encoded.push(false);
    }

    let mut carrier_bits = std::array::from_fn::<BitVec<usize, Lsb0>, CARRIERS, _>(|_| BitVec::new());

    // interleave bits
    for chunk in hamming_encoded.chunks(CARRIERS) {
        for c in 0..CARRIERS {
            carrier_bits[c].push(chunk[c]);
        }
    }

    for car in &mut carrier_bits {
        while car.len() % 4 != 0 {
            car.push(false);
        }
    }

    let mut ccs = [ConvCode::default(); CARRIERS];
    let mut y12s = [0; CARRIERS];

    let mut result = Vec::new();
    for _ in 0..2 {
        result.push([Complex::new(4, 0); CARRIERS]);
    }

    for idx in 0..carrier_bits[0].len().div_ceil(4) {
        let mut symbol = [Complex::new(0, 0); CARRIERS];
        for c in 0..CARRIERS {
            let q1234 = carrier_bits[c][idx*4..idx*4+4].load_le::<u8>();
            let q12 = q1234 >> 2;
            let q34 = q1234 & 0x3;

            y12s[c] = diff_enc(y12s[c], q12);
            let y0 = ccs[c].step(y12s[c]);

            let y01234 = (y0 << 4) | (y12s[c] << 2) | q34;

            symbol[c] = SYMBOL_TO_CONSTELLATION[y01234 as usize];
        }

        result.push(symbol);
    }

    result
}

#[derive(Default, Clone, Copy)]
struct ConvCode { a: u8, b: u8, c: u8 }

impl ConvCode {
    pub fn step(&mut self, y12: u8) -> u8 {
        let y1 = (y12 >> 1) & 1;
        let y2 = y12 & 1;

        let ap = self.c;
        let bp = self.a ^ y1 ^ y2 ^ (self.c * (self.b ^ y2));
        let cp = self.b ^ y2 ^ (y1 * self.c);
        *self = ConvCode { a: ap, b: bp, c: cp };

        // TODO: Should this be current or previous state?
        self.c
    }
}

pub fn diff_enc(y_prev: u8, q: u8) -> u8 {
  const TABLE: [[u8; 4]; 4] = [
    [0, 1, 2, 3],
    [1, 0, 3, 2],
    [2, 3, 1, 0],
    [3, 2, 0, 1]
  ];

  TABLE[y_prev as usize][q as usize]
}

#[cfg(test)]
mod test {
    use std::iter::zip;

    use super::*;

    #[test]
    pub fn test_bits_to_constellation() {
        let result = bytes_to_constellation(b"Somebody once told me the world is gonna roll me.");

        const EXPECTED: &[[(i32, i32); CARRIERS]] = &[
            [( 4,  0), ( 4,  0), ( 4,  0), ( 4,  0), ( 4,  0), ( 4,  0), ( 4,  0), ( 4,  0), ( 4,  0), ( 4,  0), ( 4,  0), ( 4,  0), ],
            [( 4,  0), ( 4,  0), ( 4,  0), ( 4,  0), ( 4,  0), ( 4,  0), ( 4,  0), ( 4,  0), ( 4,  0), ( 4,  0), ( 4,  0), ( 4,  0), ],
            [( 2,  3), (-1,  2), (-1, -2), ( 2, -1), ( 2, -1), ( 3, -2), (-1,  0), ( 4,  1), ( 2,  3), (-1,  4), (-1, -2), (-1, -4), ],
            [(-2, -3), (-2, -1), ( 1, -2), (-3,  2), (-2, -3), (-2, -1), (-3,  0), ( 2,  3), (-4, -1), ( 1,  4), (-3,  2), ( 4,  1), ],
            [( 0, -3), ( 3, -2), (-1,  0), ( 3,  2), ( 3,  0), (-1,  2), (-3, -2), ( 2, -3), ( 1, -2), (-1, -4), ( 4, -1), (-1,  4), ],
            [(-1,  4), (-1, -2), ( 2, -1), ( 0, -3), ( 2,  1), ( 0,  1), ( 0,  1), (-2, -1), (-1,  2), (-3,  2), ( 3, -2), (-2, -1), ],
            [(-3,  0), ( 1, -4), (-3, -2), ( 1,  0), (-3,  2), (-1,  2), ( 0, -1), (-4, -1), ( 1,  0), ( 4,  1), ( 3, -2), ( 3, -2), ],
            [(-2,  3), (-1,  2), (-2,  3), ( 2,  1), (-1, -4), (-1,  2), ( 3,  0), ( 1,  0), ( 2, -3), ( 2,  1), ( 4,  1), (-2, -3), ],
            [(-3,  0), ( 2,  3), (-2,  3), ( 2,  1), ( 4,  1), ( 1,  0), (-4,  1), (-1, -4), (-2,  3), (-3,  2), ( 2, -3), ( 1,  0), ],
            [(-2,  1), (-3, -2), (-2, -1), ( 1, -2), ( 1,  4), (-3,  0), (-1, -4), ( 0, -3), ( 0, -3), (-2, -3), (-3, -2), (-1,  2), ],
            [(-2,  1), (-2, -1), (-3,  0), ( 2, -3), ( 2, -3), ( 4,  1), (-2, -1), ( 1,  0), ( 2, -1), ( 1,  4), ( 3, -2), ( 2,  1), ],
            [( 1, -4), ( 2,  3), (-2,  3), ( 4, -1), ( 1,  4), ( 1,  4), ( 3, -2), (-2, -3), ( 2, -3), ( 2,  1), ( 0,  1), (-1,  0), ],
            [(-3,  2), (-1, -4), (-1,  0), ( 4,  1), ( 2, -3), ( 0,  1), ( 1, -4), ( 0,  3), ( 3,  2), (-3,  2), ( 1,  0), (-2,  3), ],
            [(-3,  2), (-3,  0), (-2, -1), ( 2,  3), (-2, -3), ( 3,  2), (-3,  2), (-1,  4), (-4, -1), ( 1, -2), (-1,  2), (-2, -1), ],
            [( 3, -2), (-4, -1), ( 4,  1), ( 1,  0), (-3,  2), ( 3, -2), ( 0,  1), ( 1, -4), (-3,  2), ( 3,  0), (-3,  0), ( 3, -2), ],
            [( 3,  0), ( 2,  3), ( 4, -1), ( 1, -2), (-1, -2), (-2, -3), ( 1,  4), (-1, -4), ( 4, -1), (-3,  2), ( 0,  1), ( 4, -1), ],
            [(-1, -4), (-2,  3), ( 4, -1), ( 1, -2), (-1,  2), ( 3,  0), (-3,  0), ( 3,  0), ( 4, -1), (-4,  1), ( 1, -2), ( 0,  3), ],
        ];

        assert_eq!(result.len(), EXPECTED.len());

        for (act, exp) in zip(result, EXPECTED) {
            for (act_iq, exp_iq) in zip(act, exp) {
                assert_eq!((act_iq.re, act_iq.im), *exp_iq);
            }
        }
    }
}