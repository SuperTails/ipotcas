def hamming_decode_15_11(chunk):
    p0 = (chunk & 0x5555).bit_count() & 1;
    p1 = (chunk & 0x6666).bit_count() & 1;
    p2 = (chunk & 0x7878).bit_count() & 1;
    p3 = (chunk & 0x7F80).bit_count() & 1;

    p = ((p3 << 3) | (p2 << 2) | (p1 << 1) | (p0 << 0))
    if p != 0:
        print(f'flipping {p - 1}')
        chunk ^= 1 << (p - 1)
    
    return (
        ((chunk >> 4) & 0x7F0) |
        ((chunk >> 3) & 0x00E) |
        ((chunk >> 2) & 0x001)
    )

def hamming_encode_15_11(val):
    chunk = (
        ((val & 0x001) << 2) |
        ((val & 0x00E) << 3) |
        ((val & 0x7F0) << 4)
    )

    p0 = (chunk & 0x5555).bit_count() & 1
    p1 = (chunk & 0x6666).bit_count() & 1
    p2 = (chunk & 0x7878).bit_count() & 1
    p3 = (chunk & 0x7F80).bit_count() & 1
    chunk ^= p0 << 0
    chunk ^= p1 << 1
    chunk ^= p2 << 3
    chunk ^= p3 << 7

    return chunk

def hamming_decode_7_4(chunk):
    p0 = (chunk & 0x5555).bit_count() & 1;
    p1 = (chunk & 0x6666).bit_count() & 1;
    p2 = (chunk & 0x7878).bit_count() & 1;

    p = ((p2 << 2) | (p1 << 1) | (p0 << 0))
    if p != 0:
        chunk ^= 1 << (p - 1)
    
    return (
        ((chunk >> 3) & 0x00E) |
        ((chunk >> 2) & 0x001)
    )

def hamming_encode_7_4(val):
    chunk = (
        ((val & 0x001) << 2) |
        ((val & 0x00E) << 3)
    )

    p0 = (chunk & 0x5555).bit_count() & 1
    p1 = (chunk & 0x6666).bit_count() & 1
    p2 = (chunk & 0x7878).bit_count() & 1
    chunk ^= p0 << 0
    chunk ^= p1 << 1
    chunk ^= p2 << 3

    return chunk