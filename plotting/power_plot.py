from plotutil import *
from codec import *
import numpy as np


NUM_SAMP = 12_000

x = np.linspace(0.0, NUM_SAMP / ADC_SAMPLE_RATE, num=NUM_SAMP, endpoint=False)


fig1, (ax_sig, ax_pwr, ax_ptot) = plt.subplots(nrows=3, sharex=True, num=1)
ax_sig.set_ylim(bottom=-1, top=1)
ax_pwr.set_ylim(bottom=-0.001, top=0.02)
#ax_ptot.set_ylim(bottom=-0.001, top=0.02)
ax_ptot.set_ylim(bottom=-0.1, top=0.1)
(ln_sig,) = ax_sig.plot(x, np.zeros_like(x), animated=True)
ln_pwr = []
for i, freq in enumerate(CARRIER_FREQS):
    ln_pwr.append(ax_pwr.plot(x, np.zeros_like(x), animated=True, label=f'{freq}')[0])
(ln_ptot,) = ax_ptot.plot(x, np.zeros_like(x), animated=True)
ax_pwr.legend()
bm1 = BlitManager(fig1.canvas, [ln_sig, *ln_pwr, ln_ptot])
#bm3 = BlitManager(fig3.canvas, [ln_power])
# make sure our window is on the screen and drawn
plt.show(block=False)
plt.pause(.1)

p = WaveSource('/dev/tty.usbmodem1401', in_rate=ADC_SAMPLE_RATE, out_rate=ADC_SAMPLE_RATE)

k = 0

while True:
    sample_idx, samples = p.get_samples(NUM_SAMP)

    k += 1

    times = (np.linspace(0.0, NUM_SAMP, num=NUM_SAMP, endpoint=False) + sample_idx) / ADC_SAMPLE_RATE
    freqs = np.array(CARRIER_FREQS).reshape((-1, 1))
    iq_comp = samples * np.exp(2j * math.pi * freqs * times)
    window_filt = np.concatenate([np.zeros(WINDOW_SIZE-1), np.ones(WINDOW_SIZE)]) / WINDOW_SIZE
    for i in range(len(CARRIER_FREQS)):
        iq_comp[i] = np.convolve(iq_comp[i], window_filt, 'same')
    iq_mag_sq = np.square(np.absolute(iq_comp))
    
    ptot = np.sum(iq_mag_sq, axis=0)

    if k < 10:
        ln_sig.set_ydata(samples)
        for i, l in enumerate(ln_pwr):
            l.set_ydata(iq_mag_sq[i])
        ln_ptot.set_ydata(np.real(iq_comp[0]))

    bm1.update()