from serial import Serial
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np
import time
import math
from plotutil import *
from numpy import fft
from scipy import signal
from codec import *

PLOT_SAMPLES = 32_000
PLOT_TIME = PLOT_SAMPLES / ADC_SAMPLE_RATE

SYMBOL_PERIOD = 5e-3
SAMPLES_PER_SYMBOL = int(SYMBOL_PERIOD * ADC_SAMPLE_RATE)

x = np.linspace(0.0, PLOT_SAMPLES / ADC_SAMPLE_RATE, PLOT_SAMPLES, endpoint=False)

p = WaveSource('/dev/tty.usbmodem1401', in_rate=ADC_SAMPLE_RATE, out_rate=ADC_SAMPLE_RATE)

i_correct = []
q_correct = []
for s in range(len(SYMBOL_TO_CONSTELLATION)):
    i_correct.append(SYMBOL_TO_CONSTELLATION[s][0])
    q_correct.append(SYMBOL_TO_CONSTELLATION[s][1])

# make a new figure
fig1, (ax_sig, ax_iq) = plt.subplots(nrows=2, sharex=True, num=1)

# add a line
ax_sig.set_ylim(bottom=-1.0, top=1.0)
(ln_sig,) = ax_sig.plot(x, np.zeros_like(x), animated=True)

ax_iq.set_ylim(bottom=-4.5, top=4.5)
ax_iq.set_yticks([-4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0])
ax_iq.grid(visible=True, which='major', axis='y')
(ln_i,) = ax_iq.plot(x, np.zeros_like(x), animated=True, label='I')
(ln_q,) = ax_iq.plot(x, np.zeros_like(x), animated=True, label='Q')
(ln_p,) = ax_iq.plot(x, np.zeros_like(x), animated=True, label='P')
ln_iq_samples = ax_iq.scatter(x, np.zeros_like(x), animated=True, label='Samples')
ax_iq.legend()


# add a frame number
fr_number = ax_sig.annotate(
    "0",
    (0, 1),
    xycoords="axes fraction",
    xytext=(10, -10),
    textcoords="offset points",
    ha="left",
    va="top",
    animated=True,
)

fig2, (ax_cons) = plt.subplots(num=2)
fig2.subplots_adjust(bottom=0.25)
ax_cons.set_xlim(left=-5, right=5)
ax_cons.set_ylim(bottom=-5, top=5)
ax_cons.scatter(i_correct, q_correct)
ln_cons = ax_cons.scatter(i_correct, q_correct, animated=True)

allowed_freqs = CARRIER_FREQS

MONITOR_FREQ = CARRIER_FREQS[0]

def on_freq_change(val):
    global MONITOR_FREQ
    MONITOR_FREQ = val

ax_freq = fig2.add_axes([0.25, 0.15, 0.65, 0.03])
freq_slider = Slider(
    ax_freq, "Freq", CARRIER_FREQS[0], CARRIER_FREQS[-1],
    valinit=CARRIER_FREQS[0], valstep=CARRIER_FREQS
)
freq_slider.on_changed(on_freq_change)

#fig3, (ax_power) = plt.subplots(num=3)
#(ln_power,) = ax_power.semilogy(x, np.ones_like(x))

bm1 = BlitManager(fig1.canvas, [ln_sig, fr_number, ln_i, ln_q, ln_p, ln_iq_samples])
bm2 = BlitManager(fig2.canvas, [ln_cons])
#bm3 = BlitManager(fig3.canvas, [ln_power])
# make sure our window is on the screen and drawn
plt.show(block=False)
plt.pause(.1)

p = WaveSource('/dev/tty.usbmodem1401', in_rate=ADC_SAMPLE_RATE, out_rate=ADC_SAMPLE_RATE)

t = time.perf_counter()


plot_samples = np.zeros(PLOT_SAMPLES)

POWER_WINDOW_SIZE = WINDOW_SIZE * 2

SAMPLE_OFFSET = 50

pow_data = np.zeros(PLOT_SAMPLES // POWER_WINDOW_SIZE)

abc = 0

l = 0
while True:
    k = 0

    SEARCH_SAMPLES = 64_000

    t, samples = p.get_samples(SEARCH_SAMPLES)
    x = np.linspace(t, t + SEARCH_SAMPLES, SEARCH_SAMPLES, endpoint=False) / ADC_SAMPLE_RATE

    iq_values = mix_and_filt(samples, CARRIER_FREQS, ADC_SAMPLE_RATE)
    print('SHAPE: ', iq_values.shape)
    pwr = np.sum(np.square(np.absolute(iq_values)), axis=0)

    pwr_thresh = pwr < 1e-4
    pwr_thresh[:100] = False

    (quiet_indices,) = np.nonzero(pwr_thresh)
    first_quiet = quiet_indices[0]
    print('FIRST QUIET: ', first_quiet / ADC_SAMPLE_RATE)

    samples = samples[first_quiet:]
    iq_values = iq_values[:,first_quiet:]
    pwr = pwr[first_quiet:]

    pwr_thresh2 = pwr > 1e-2
    (loud_indices,) = np.nonzero(pwr_thresh2)
    first_loud = loud_indices[0]

    samples = samples[first_loud:first_loud+PLOT_SAMPLES]
    iq_values = iq_values[:,first_loud:first_loud+PLOT_SAMPLES]
    pwr = pwr[first_loud:first_loud+PLOT_SAMPLES]

    print(decode(samples, CARRIER_FREQS, ADC_SAMPLE_RATE, SYMBOL_PERIOD, skip=2, sample_offset=SAMPLE_OFFSET))

    sample_times = symbol_sample_indices(len(samples), SYMBOL_PERIOD, ADC_SAMPLE_RATE, sample_offset=SAMPLE_OFFSET) / ADC_SAMPLE_RATE

    print('SAMPLE TIMES: ', sample_times)

    #phase = np.angle(iq_lpf[SAMPLES_PER_SYMBOL * 3 // 2])
    #mag = np.abs(iq_lpf[SAMPLES_PER_SYMBOL * 3 // 2])
    #print(phase, mag)

    #iq_lpf = iq_lpf * (np.exp(-1j * phase) * 4.0 / mag)

    adjusts = 4.0 / iq_values[:,int(SAMPLES_PER_SYMBOL*2-SAMPLE_OFFSET)]
    #adjusts = np.array([4.0 + 0j for _ in range(len(CARRIER_FREQS))])
    freq_idx = CARRIER_FREQS.index(MONITOR_FREQ)

    cons_points = iq_values * adjusts.reshape((len(CARRIER_FREQS), 1))

    cons_samples = cons_points[freq_idx,SAMPLES_PER_SYMBOL*(2+1) - SAMPLE_OFFSET::SAMPLES_PER_SYMBOL].copy()

    sample_times = sample_times.reshape((len(sample_times), 1))

    ln_iq_samples.set_offsets(np.concatenate([sample_times, np.zeros((len(sample_times), 1))], axis=1))

    # update the artists
    #ln.set_xdata(x)
    ln_sig.set_ydata(samples)
    ln_i.set_ydata(np.real(cons_points[freq_idx]))
    ln_q.set_ydata(np.imag(cons_points[freq_idx]))
    # From https://stackoverflow.com/questions/38900344/convert-complex-numpy-array-into-n-2-array-of-real-and-imaginary-parts
    ln_cons.set_offsets(cons_samples.view(float).reshape(-1, 2))
    ln_p.set_ydata(pwr * 1e2)
    fr_number.set_text(f"time: {t / ADC_SAMPLE_RATE}")
    # tell the blitting manager to do its thing
    bm1.update()
    bm2.update()

    t3 = time.perf_counter()