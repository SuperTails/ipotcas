import numpy as np
from numpy import fft
import matplotlib.pyplot as plt
from codec import *

ENCODED_DATA = b"Somebody once told me the world is gonna roll me / I ain't the sharpest tool in the shed / She was looking kind of dumb with her finger and her thumb / In the shape of an \"L\" on her forehead / Well, the years start comin' and they don't stop comin' / Fed to the rules and I hit the ground runnin' / Didn't make sense not to live for fun / Your brain gets smart but your head gets dumb / So much to do, so much to see / So what's wrong with taking the backstreets? / You'll never know if you don't go / You'll never shine if you don't glow"
enc = bits_to_wave(ENCODED_DATA, freqs=CARRIER_FREQS, sample_rate=32e3, symbol_period=5e-3, train_periods=2)

a = np.load('samples.npy')

freqs_y = fft.fft(a)
freqs_x = fft.fftfreq(n=len(a), d=1/32e3)

enc_y = fft.fft(enc)
enc_x = fft.fftfreq(n=len(enc), d=1/32e3)

plt.figure(1)
plt.plot(a)

plt.figure(2)
plt.plot(freqs_x, np.abs(freqs_y))

plt.figure(3)
plt.plot(enc_x, np.abs(enc_y))

plt.figure(4)
plt.plot(enc)

plt.show()