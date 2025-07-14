#![allow(clippy::identity_op)] // I like the symmetry

use bitvec::{field::BitField, order::Lsb0, slice::BitSlice, vec::BitVec, view::BitView};

pub fn encode(bv: &BitSlice<u8>) -> BitVec {
    let mut result = BitVec::new();
    for chunk in bv.chunks(4) {
        let enc = encode_7_4(chunk.load_le::<u8>());
        result.extend_from_bitslice(&enc.view_bits::<Lsb0>()[0..7]);
    }
    result
}

pub fn decode(bv: &BitSlice<u8>) -> BitVec {
    let mut result = BitVec::new();
    for chunk in bv.chunks_exact(7) {
        let enc = decode_7_4(chunk.load_le::<u8>());
        result.extend_from_bitslice(&enc.view_bits::<Lsb0>()[0..4]);
    }
    result
}

pub fn decode_7_4(mut chunk: u8) -> u8 {
    let p0 = (chunk & 0x55).count_ones() & 1;
    let p1 = (chunk & 0x66).count_ones() & 1;
    let p2 = (chunk & 0x78).count_ones() & 1;

    let p = (p2 << 2) | (p1 << 1) | (p0 << 0);
    if p != 0 {
        chunk ^= 1 << (p - 1)
    }
    
    ((chunk >> 3) & 0x00E) | ((chunk >> 2) & 0x001)
}

pub fn encode_7_4(val: u8) -> u8 {
    let mut chunk = 
        ((val & 0x001) << 2) |
        ((val & 0x00E) << 3);

    let p0 = (chunk & 0x55).count_ones() as u8 & 1;
    let p1 = (chunk & 0x66).count_ones() as u8 & 1;
    let p2 = (chunk & 0x78).count_ones() as u8 & 1;
    chunk ^= p0 << 0;
    chunk ^= p1 << 1;
    chunk ^= p2 << 3;

    chunk
}