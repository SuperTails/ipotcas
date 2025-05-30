from serial import Serial
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib import collections as mc
import numpy as np
import time
import math
from threading import Thread
from queue import Queue
from plotutil import *
from numpy import fft
from scipy import signal
from codec import *

PLOT_SAMPLES = 10_000
PLOT_TIME = PLOT_SAMPLES / ADC_SAMPLE_RATE

SAMPLES_PER_SYMBOL = int(SYMBOL_PERIOD * ADC_SAMPLE_RATE)

x = np.linspace(0.0, PLOT_SAMPLES / ADC_SAMPLE_RATE, PLOT_SAMPLES, endpoint=False)

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

ax_iq.set_ylim(bottom=-5.5, top=5.5)
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

TEST_DATA = (1507).to_bytes(2, 'little') + b''.join([
    b"Somebody once told me the world is gonna roll me / ",
    b"I ain't the sharpest tool in the shed / ",
    b"She was looking kind of dumb with her finger and her thumb / ",
    b"In the shape of an \"L\" on her forehead / ",
    b"Well, the years start comin' and they don't stop comin' / ",
    b"Fed to the rules and I hit the ground runnin' / ",
    b"Didn't make sense not to live for fun / ",
    b"Your brain gets smart but your head gets dumb / ",
    b"So much to do, so much to see / ",
    b"So what's wrong with taking the backstreets? / ",
    b"You'll never know if you don't go / ",
    b"You'll never shine if you don't glow / ",
    b"Hey now, you're an all star / ",
    b"Get your game on, go play / ",
    b"Hey now, you're a rock star / ",
    b"Get your show on, get paid / ",
    b"(And all that glitters is gold) / ",
    b"Only shootin' stars break the mold",
])

TEST_CONS_POINTS = bits_to_constellation(TEST_DATA, len(CARRIER_FREQS), 0)

cons_errors = np.tile([[np.random.random(), np.random.random()], [1, 1]], (100, 1, 1))
ln_cons_errs = mc.LineCollection(cons_errors)
ln_cons_errs.set_snap(False)
ln_cons_errs.set_animated(True)

fig2, (ax_cons) = plt.subplots(num=2)
fig2.subplots_adjust(bottom=0.25)
ax_cons.set_xlim(left=-5, right=5)
ax_cons.set_ylim(bottom=-5, top=5)
ax_cons.scatter(i_correct, q_correct)
ax_cons.add_collection(ln_cons_errs)
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
bm2 = BlitManager(fig2.canvas, [ln_cons, ln_cons_errs])
#bm3 = BlitManager(fig3.canvas, [ln_power])
# make sure our window is on the screen and drawn
plt.show(block=False)
plt.pause(.1)

p = WaveSource('/dev/tty.usbmodem1301', in_rate=ADC_SAMPLE_RATE, out_rate=ADC_SAMPLE_RATE)
#p = WaveSource('/dev/tty.usbmodem1303', in_rate=DAC_SAMPLE_RATE, out_rate=ADC_SAMPLE_RATE)

t = time.perf_counter()

# [1.6120351  1. 1.63643268 1.63171056 2.81253739 2.87263723 6.11071582 3.35473016 ]
# [1.35503962 1. 4.20434687 2.37828665 3.08321257 3.55501741 2.56564467 4.58585968 ]

plot_samples = np.zeros(PLOT_SAMPLES)

POWER_WINDOW_SIZE = WINDOW_SIZE * 2

SAMPLE_OFFSET = 15

pow_data = np.zeros(PLOT_SAMPLES // POWER_WINDOW_SIZE)

abc = 0

SEARCH_SAMPLES = 80_000

t_start, samples = p.get_samples(SEARCH_SAMPLES)

segment_queue = Queue()

def get_segments():
    samples = np.array([0.0])

    def refill_buffer():
        nonlocal samples
        while len(samples) < SEARCH_SAMPLES:
            _, new_samples = p.get_samples(SEARCH_SAMPLES)
            samples = np.concatenate([samples, new_samples], axis=0)

    while True:
        refill_buffer()

        print('S: ', samples.shape)

        iq_values = mix_and_filt(samples, CARRIER_FREQS, ADC_SAMPLE_RATE)
        pwr = np.sum(np.square(np.absolute(iq_values)), axis=0)

        pwr_thresh = pwr < 1e-4
        #pwr_thresh = pwr < 1e-9
        pwr_thresh[:100] = False

        (quiet_indices,) = np.nonzero(pwr_thresh)

        first_quiet = quiet_indices[0]
        print('FIRST QUIET: ', first_quiet / ADC_SAMPLE_RATE)

        samples = samples[first_quiet:]

        refill_buffer()

        iq_values = mix_and_filt(samples, CARRIER_FREQS, ADC_SAMPLE_RATE)
        pwr = np.sum(np.square(np.absolute(iq_values)), axis=0)

        pwr_thresh2 = pwr > 1e-3
        #pwr_thresh2 = pwr > 1e-4
        (loud_indices,) = np.nonzero(pwr_thresh2)
        first_loud = loud_indices[0]

        samples = samples[first_loud:]

        segment_queue.put(samples[:PLOT_SAMPLES])
        samples = samples[PLOT_SAMPLES:]

segment_thread = Thread(target=get_segments)
segment_thread.start()

l = 0
while True:
    k = 0

    samples = segment_queue.get()

    plot_samples = samples[:PLOT_SAMPLES]

    print('SAMP SHAPE: ', plot_samples.shape, samples.shape)

    print(decode(plot_samples, CARRIER_FREQS, ADC_SAMPLE_RATE, SYMBOL_PERIOD, skip=2, sample_offset=SAMPLE_OFFSET))

    sample_times = symbol_sample_indices(len(plot_samples), SYMBOL_PERIOD, ADC_SAMPLE_RATE, sample_offset=SAMPLE_OFFSET) / ADC_SAMPLE_RATE

    print('SAMPLE TIMES: ', sample_times)

    #phase = np.angle(iq_lpf[SAMPLES_PER_SYMBOL * 3 // 2])
    #mag = np.abs(iq_lpf[SAMPLES_PER_SYMBOL * 3 // 2])
    #print(phase, mag)

    #iq_lpf = iq_lpf * (np.exp(-1j * phase) * 4.0 / mag)

    iq_values = mix_and_filt(plot_samples, CARRIER_FREQS, ADC_SAMPLE_RATE)
    pwr = np.sum(np.square(np.absolute(iq_values)), axis=0)
    adjusts = 4.0 / iq_values[:,int(SAMPLES_PER_SYMBOL*2-SAMPLE_OFFSET)]
    #adjusts = np.array([4.0 + 0j for _ in range(len(CARRIER_FREQS))])
    freq_idx = CARRIER_FREQS.index(MONITOR_FREQ)

    cons_points = iq_values * adjusts.reshape((len(CARRIER_FREQS), 1))
    cons_samples = cons_points[:,SAMPLES_PER_SYMBOL*(2+1) - SAMPLE_OFFSET::SAMPLES_PER_SYMBOL]
    phase_adj = np.zeros_like(cons_samples, dtype=np.complex128)

    avg_drift2 = 0.0
    for i in range(cons_samples.shape[1]):
        cons_samples[:,i] *= np.exp(-1j * avg_drift2 * np.array(CARRIER_FREQS))
        avg_drift = 0.0
        for f in range(cons_samples.shape[0]):
            iq, _ = nearest_iq(cons_samples[f][i])
            avg_drift += np.angle(cons_samples[f][i] / iq) / CARRIER_FREQS[f] / len(CARRIER_FREQS)
        avg_drift2 += avg_drift
        #phase_adj[:,i] = np.exp(-1j * avg_drift * np.array(CARRIER_FREQS))
        #print('AVG: ', math.degrees(avg_drift * CARRIER_FREQS[-1]))
    
    #cons_samples = cons_samples * phase_adj

    cons_count = min(TEST_CONS_POINTS.shape[0], cons_samples.shape[1])
    correct_points = TEST_CONS_POINTS[:cons_count,:]
    received_points = np.transpose(cons_samples)[:cons_count,:]

    cons_distances = np.average(np.absolute(correct_points - received_points), axis=0)
    cons_offsets = np.average(correct_points / received_points, axis=0)

    angular_vel = 1e6 * np.angle((correct_points / received_points / np.linspace(0.0, SYMBOL_PERIOD * cons_count, num=cons_count, endpoint=False).reshape((cons_count, 1)))[1:]) / np.array(CARRIER_FREQS).reshape((1,8))

    print(TEST_CONS_POINTS[:5,0])
    print(np.transpose(cons_samples)[:5,0])

    print('DISTANCES: ', cons_distances)
    print('OFFSETS: ', np.abs(cons_offsets), np.angle(cons_offsets))
    print('ANGULAR VEL: ', np.average(angular_vel, axis=0))
    print('STDDEV: ', np.std(angular_vel, axis=0))

    sample_times = sample_times.reshape((len(sample_times), 1))

    ln_iq_samples.set_offsets(np.concatenate([sample_times, np.zeros((len(sample_times), 1))], axis=1))

    correct_points_iq = correct_points[:,freq_idx].copy().view(float).reshape(-1,1,2)
    received_points_iq = received_points[:,freq_idx].copy().view(float).reshape(-1,1,2)

    errs = np.concatenate([correct_points_iq, received_points_iq], axis=1)
    print('ERRS SHAPE ', errs.shape)

    # update the artists
    #ln.set_xdata(x)
    ln_sig.set_ydata(plot_samples)
    ln_i.set_ydata(np.real(cons_points[freq_idx]))
    ln_q.set_ydata(np.imag(cons_points[freq_idx]))
    # From https://stackoverflow.com/questions/38900344/convert-complex-numpy-array-into-n-2-array-of-real-and-imaginary-parts
    ln_cons.set_offsets(cons_samples[freq_idx].copy().view(float).reshape(-1, 2))
    #ln_cons_errs.set_segments(errs)
    ln_p.set_ydata(pwr * 1e4)
    fr_number.set_text(f"time: {t / ADC_SAMPLE_RATE}")
    # tell the blitting manager to do its thing
    bm1.update()
    bm2.update()


    t3 = time.perf_counter()