import wave
import struct
from serial import Serial

wavfile = wave.open('recording.wav', 'w')
wavfile.setnchannels(1)
wavfile.setsampwidth(2)
wavfile.setframerate(48_000)

usb_p = Serial('/dev/tty.usbmodem1201', 1_000_000)

msg_count = 0

try:
    while True:
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
        body = usb_p.read(l)
        if msg_type == 1:
            wavfile.writeframes(body)
        elif msg_type != 1:
            print(msg_type)

except KeyboardInterrupt:
    pass
