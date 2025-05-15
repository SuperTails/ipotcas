import copy
import math
import matplotlib.pyplot as plt
import numpy as np
import sys
from typing import Optional
from numpy import fft
from scipy import signal
from dataclasses import dataclass

def hamming_dist(lhs, rhs, num_bits):
    result = 0
    for i in range(num_bits):
        if (lhs ^ rhs) & (1 << i):
            result += 1
    return result

def signed_angle_diff(lhs, rhs):
    return (lhs - rhs + 3.0 * math.pi) % (2.0 * math.pi) - math.pi
 

# From the v.32 specification
CONSTELLATION_TABLE = [
       None,    None,    None, 0b11111,    None, 0b11000,    None,    None,    None,
       None,    None, 0b01000,     'D', 0b00101,    None, 0b01010,    None,    None,
       None, 0b10010,    None, 0b10101,    None, 0b10011,    None, 0b10100,    None,
    0b00000,    None, 0b01111,    None, 0b00010,    None, 0b01101,     'C', 0b00011,
       None, 0b11001,    None, 0b11110,    None, 0b11010,    None, 0b11101,    None,
    0b00111,     'A', 0b01001,    None, 0b00110,    None, 0b01011,    None, 0b00100,
       None, 0b10000,    None, 0b10111,    None, 0b10001,    None, 0b10110,    None,
       None,    None, 0b01110,    None, 0b00001,     'B', 0b01100,    None,    None,
       None,    None,    None, 0b11100,    None, 0b11011,    None,    None,    None,
]

SYMBOL_TO_CONSTELLATION = []

CONSTELLATION_TO_SYMBOL = dict()

for i in range(0b1_00000):
    idx = CONSTELLATION_TABLE.index(i)
    row, col = idx // 9, idx % 9
    cons_i = col - 4
    cons_q = 4 - row
    CONSTELLATION_TO_SYMBOL[(cons_i, cons_q)] = i
    SYMBOL_TO_CONSTELLATION.append((cons_i, cons_q))

def diff_enc(y_prev, q):
    TABLE = [
        [0, 1, 2, 3],
        [1, 0, 3, 2],
        [2, 3, 1, 0],
        [3, 2, 0, 1],
    ]

    return TABLE[y_prev][q]
    #return (q + y_prev) % 4

def diff_dec(y_prev, y_new):
    TABLE = [
        [0, 1, 2, 3],
        [1, 0, 3, 2],
        [3, 2, 0, 1],
        [2, 3, 1, 0],
    ]

    return TABLE[y_prev][y_new]
    #return (y_new - y_prev + 4) % 4

class ConvCode:
    a: int
    b: int
    c: int

    def __init__(self):
        self.a = 0
        self.b = 0
        self.c = 0
    
    def step(self, y12):
        result = self.c

        y1 = (y12 >> 1) & 1
        y2 = y12 & 1

        ap = self.c
        bp = self.a ^ y1 ^ y2 ^ (self.c * (self.b ^ y2))
        cp = self.b ^ y2 ^ (y1 * self.c)
        self.a, self.b, self.c = ap, bp, cp

        # TODO: Should this be current or previous state?
        return self.c

def find_cc_table():
    table = []

    print('CONV CODE:')
    for c in range(8):
        sub_table = []
        print(f'{c:03b} -> ', end='')
        for i in range(4):
            cc = ConvCode()
            cc.a = (c >> 2) & 1
            cc.b = (c >> 1) & 1
            cc.c = (c >> 0) & 1
            cc.step(i)
            # TODO: Will need to change this if I change prev/current state
            c2 = (cc.a << 2) | (cc.b << 1) | (cc.c)
            sub_table.append(c2)
            print(f'{c2:03b} ', end='')
        print()
        table.append(sub_table)
    
    return table

CC_TABLE = find_cc_table()

@dataclass
class BitString:
    data: int
    bits: int

    @classmethod
    def from_bytes(cls, bytestring: bytes):
        data = 0
        bits = 0
        for b in bytestring:
            data |= b << bits
            bits += 8
        return cls(data, bits)
    
    def to_bytes(self):
        assert self.bits % 8 == 0
        return self.data.to_bytes(self.bits // 8, 'little')


    def push_msb(self, b):
        self.data |= b.data << self.bits
        self.bits += b.bits

    def pop_lsb(self, count = 1, strict=True):
        if strict:
            assert 0 <= count <= self.bits

        result = self.data & ((1 << count) - 1)
        self.data >>= count
        if count <= self.bits:
            self.bits -= count
        else:
            self.bits = 0
        return result
   
def raised_cos_filt(t, rolloff):
    if abs(t) < 1e-3:
        return 1.0
    if rolloff > 1e-3 and abs(abs(t) - (0.5 / rolloff)) < 1e-3:
        return rolloff * 0.5 * math.sin(math.pi * 0.5 / rolloff)
    
    numer = math.sin(math.pi * t) * math.cos(math.pi * rolloff * t)
    denom = math.pi * t * (1.0 - pow(2.0 * rolloff * t, 2))
    return numer / denom

def bits_to_constellation(d, num_carriers, train_periods=2):
    if isinstance(d, bytes):
        d = BitString.from_bytes(d)
    elif isinstance(d, BitString):
        d = copy.copy(d)
    else:
        raise Exception()

    num_symbols = math.ceil(d.bits / 4 / num_carriers)

    iq_values = np.zeros((num_symbols + train_periods, num_carriers), dtype=np.complex128)

    iq_values[:train_periods] = 4 + 0j

    ccs = [ConvCode() for _ in range(num_carriers)]
    y12s = [0 for _ in range(num_carriers)]

    t = train_periods
    while d.bits > 0:
        for f in range(num_carriers):
            q34 = d.pop_lsb(2, strict=False)
            q12 = d.pop_lsb(2, strict=False)

            y12s[f] = diff_enc(y12s[f], q12)
            y0 = ccs[f].step(y12s[f])
            
            i, q = SYMBOL_TO_CONSTELLATION[(y0 << 4) | (y12s[f] << 2) | q34]

            iq_values[t,f] = i + q * 1j

        t += 1
    
    return iq_values

def constellation_to_wave(cons, freqs, sample_rate, symbol_period):
    assert len(cons.shape) == 2
    assert cons.shape[1] == len(freqs)

    samples_per_symbol = int(symbol_period * sample_rate)
    samples = int(samples_per_symbol * cons.shape[0])

    print(f'{samples} samples')
    print(f'{samples_per_symbol} samples per symbol')
    print(f'{samples / sample_rate} total time')

    cons_repeated = np.repeat(cons, samples_per_symbol, axis=0)

    sample_times = np.linspace(0.0, samples/sample_rate, num=samples, endpoint=False)

    # negative sign is needed or else we get the complex conj. of the desired symbol
    phasors = np.exp(-2.0j * math.pi * np.outer(sample_times, freqs))

    carriers = np.real(cons_repeated * phasors)

    return np.sum(carriers, axis=1)

def bits_to_wave(d, freqs, sample_rate, symbol_period, train_periods=2):
    if isinstance(d, bytes):
        d = BitString.from_bytes(d)
    
    assert (isinstance(d, BitString))

    cons = bits_to_constellation(d, len(freqs), train_periods)
    return constellation_to_wave(cons, freqs, sample_rate, symbol_period)

class Trellis:
    pm: list[list[float]]
    prev: list[list[Optional[int]]]
    q34_list: list[list[Optional[int]]]

    def __init__(self):
        # Mark only the starting state, 000, as being possible initially
        self.pm = [[math.inf for _ in range(8)]]
        self.pm[0][0] = 0.0
        self.prev = []
        self.q34_list = []

    def append(self, iq):
        self.prev.append([None] * 8)
        self.pm.append([math.inf for _ in range(8)])
        self.q34_list.append([None] * 8)
        for prev_state in range(8):
            for cur_state in range(8):
                try:
                    y12_trans = CC_TABLE[prev_state].index(cur_state)
                except ValueError:
                    # This transition is impossible
                    continue

                # TODO: May need to edit this if I change prev/current state in cc
                #bm = hamming_dist(y12_new, y12_trans, 2) + hamming_dist(cur_state & 1, y0, 1)
                #bm = hamming_dist(y12_new, y12_trans, 2)

                # Find which q34 option was most likely to lead to this path
                bm = math.inf
                for guessed_q34 in range(4):
                    i_exp, q_exp = SYMBOL_TO_CONSTELLATION[((cur_state & 1) << 4) | (y12_trans << 2) | guessed_q34]
                    iq_exp = (i_exp + q_exp * 1j)
                    guessed_bm = np.abs(iq - iq_exp)
                    if guessed_bm < bm:
                        likely_q34 = guessed_q34
                        bm = guessed_bm

                # If this combination of (prev, current) was more likely to lead us to this state
                # than our previous guess, use the new guess.
                new_metric = self.pm[-2][prev_state] + bm
                if self.pm[-1][cur_state] > new_metric:
                    self.pm[-1][cur_state] = new_metric
                    self.prev[-1][cur_state] = prev_state
                    self.q34_list[-1][cur_state] = likely_q34

    def decode_most_likely(self):
        best_final_state = None
        best_final_state_score = math.inf
        for s in range(8):
            if self.pm[-1][s] < best_final_state_score:
                best_final_state_score = self.pm[-1][s]
                best_final_state = s
        assert best_final_state is not None
        
        corrected_q34 = []
        corrected_y12 = []
        cur_state = best_final_state
        for t in reversed(range(len(self.prev))):
            prev_state = self.prev[t][cur_state] #type:ignore
            corrected_q34.append(self.q34_list[t][cur_state])
            corrected_y12.append(CC_TABLE[prev_state].index(cur_state))
            cur_state = prev_state
        corrected_y12.reverse()
        corrected_q34.reverse()

        result = BitString(0, 0)
        y12 = 0
        for q34, t in zip(corrected_q34, corrected_y12):
            q12 = diff_dec(y12, t)
            y12 = t

            s = (q12 << 2) | q34
            result.push_msb(BitString(s, 4))
        
        return result
       
WINDOW_SIZE = 64

def mix_and_filt(sig, freqs, sample_rate):
    samples = sig.shape[0]

    times = np.linspace(0.0, samples/sample_rate, num=len(sig), endpoint=False)
    mixed = sig.reshape((1, samples)) * np.exp(2.0j * math.pi * np.outer(freqs, times))

    iq_filt = np.zeros_like(mixed)
    window_filt = np.concatenate([np.zeros(WINDOW_SIZE-1), np.ones(WINDOW_SIZE)]) / WINDOW_SIZE
    for f in range(len(freqs)):
        iq_filt[f,:] = np.convolve(mixed[f,:], window_filt, mode='same')
    
    return iq_filt

def trellis_decode(samples):
    trellises = [Trellis() for _ in range(samples.shape[0])]

    for f, t in enumerate(trellises):
        for s in samples[f]:
            t.append(s)
    
    streams = [t.decode_most_likely() for t in trellises]

    result = BitString(0, 0)
    done = False
    while not done:
        for s in streams:
            if s.bits == 0:
                done = True
                break
            result.push_msb(BitString(s.pop_lsb(4), 4))
    
    return result.to_bytes()


def decode(sig, freqs, sample_rate, symbol_period, skip, sample_offset=40):
    assert len(sig.shape) == 1

    samples_per_symbol = int(symbol_period * sample_rate)
    symbols = len(sig) // samples_per_symbol

    iq_filt = mix_and_filt(sig, freqs, sample_rate)

    adjusts = 4.0 / iq_filt[:,samples_per_symbol*skip-sample_offset]
    print('Adjustments: ', adjusts)

    #samp_x = []
    #for x in range(symbols):
    #    samp_x.append((x + 0.5) * symbol_period)
    #samp_y = np.zeros(len(samp_x))
    #plt.scatter(samp_x, samp_y)
    #plt.legend()

    q34_list = []

    samples = iq_filt[:,samples_per_symbol*(skip+1) - sample_offset::samples_per_symbol] * adjusts.reshape((len(freqs), 1))
    
    return trellis_decode(samples)
    
CARRIER_FREQS = [4.5e3, 5e3, 5.5e3, 6e3, 6.5e3, 7e3, 7.5e3, 8e3]
#CARRIER_FREQS = [4.5e3]

ENCODED_DATA = b"Somebody once told me the world is gonna roll me / I ain't the sharpest tool in the shed / She was looking kind of dumb with her finger and her thumb / In the shape of an \"L\" on her forehead / Well, the years start comin' and they don't stop comin' / Fed to the rules and I hit the ground runnin' / Didn't make sense not to live for fun / Your brain gets smart but your head gets dumb / So much to do, so much to see / So what's wrong with taking the backstreets? / You'll never know if you don't go / You'll never shine if you don't glow"
print('ENCODED:')
encoded = bits_to_wave(
    ENCODED_DATA,
    freqs=CARRIER_FREQS,
    sample_rate=32e3,
    symbol_period=10e-3
)
times = np.linspace(0, len(encoded) / 32e3, len(encoded), endpoint=False)

decode(
    encoded,
    freqs=CARRIER_FREQS,
    sample_rate=32e3,
    symbol_period=10e-3,
    skip=2
)