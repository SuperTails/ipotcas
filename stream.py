from serial import Serial
import numpy as np
import struct
import time
import wave

wavfile = wave.open('Bizet_Habanera.wav', 'rb')
nframes = wavfile.getnframes()

FRAME_SIZE = 288

sound_bytes = bytearray()
sound_idx = 0

for f in range(0, nframes//2 * 2, 2):
    s0, s1 = struct.unpack('<hh', wavfile.readframes(2))
    s0 >>= 4
    s1 >>= 4

    s = ((s1 & 0xFFF) << 12) | (s0 & 0xFFF)
    sound_bytes += s.to_bytes(3, 'little')

'''
for i in range(0, len(sound_buf), 2):
    s0 = int(sound_buf[i+0] * 2047.0) & 0xFFF
    s1 = int(sound_buf[i+1] * 2047.0) & 0xFFF

    
    sound_bytes += s.to_bytes(3, 'little')

assert(len(sound_bytes) == 288)
'''

uart_p = Serial('/dev/tty.usbmodem1403', 1_000_000)
#usb_p = Serial('/dev/tty.usbmodem1101', 115200)

SAMPLE_RATE = 48e3
TONE_FREQ = 1e3

#sound_buf = np.sin(np.linspace(0, 192 / SAMPLE_RATE, num=192, endpoint=False) * 2.0 * np.pi * TONE_FREQ)

#sound_bytes = bytearray()

st = time.perf_counter()

#uart_p.write(sound_bytes[sound_idx:sound_idx+FRAME_SIZE])
#sound_idx += FRAME_SIZE
#uart_p.write(sound_bytes[sound_idx:sound_idx+FRAME_SIZE])
#sound_idx += FRAME_SIZE
#uart_p.flush()

msg_count = 0
num_sent = 1

uart_b = b''

while sound_idx < len(sound_bytes):
    new_uart = uart_p.read_all()
    if new_uart is not None:
        uart_b += new_uart
        while True:
            idx = uart_b.find(b'\n')
            if idx == -1:
                break
            line, uart_b = uart_b[:idx+1], uart_b[idx+1:]
            if line == b'S\n':
                num_sent += 1
                uart_p.write(sound_bytes[sound_idx:sound_idx+FRAME_SIZE])
                sound_idx += FRAME_SIZE
                uart_p.flush()
                msg_count += 1
                if msg_count % 100 == 0:
                    print(f'Count: {msg_count}, per second: {num_sent / (time.perf_counter() - st)}')
            else:
                print(line.decode(encoding='ascii', errors='replace'), end='')

    '''
    while usb_p.read() != b'\xFF':
        pass
    if usb_p.read() != b'\x55':
        continue
    if usb_p.read() != b'\xAA':
        continue
    if usb_p.read() != b'\x00':
        continue
    
    msg_count += 1

    msg_type, l = struct.unpack('<HH', usb_p.read(4))
    usb_p.read(l)
    if msg_type == 3:
        num_sent += 1
        uart_p.write(sound_bytes[sound_idx:sound_idx+FRAME_SIZE])
        sound_idx += FRAME_SIZE
        uart_p.flush()
    elif msg_type != 1:
        print(msg_type)
    '''
    
