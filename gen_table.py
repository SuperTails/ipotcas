import math
import struct

vals = []

for i in range(256):
    c = int(128 * math.cos(i / 256 * 2 * math.pi))
    s = int(128 * math.sin(i / 256 * 2 * math.pi))
    vals.append(int.from_bytes(struct.pack('<hh', c, s), 'little'))
 
print(', '.join(f'0x{a:08X}' for a in vals))