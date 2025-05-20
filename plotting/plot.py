from serial import Serial
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np
import time
import math
from numpy import fft
from scipy import signal
from codec import *

class BlitManager:
    def __init__(self, canvas, animated_artists=()):
        """
        Parameters
        ----------
        canvas : FigureCanvasAgg
            The canvas to work with, this only works for subclasses of the Agg
            canvas which have the `~FigureCanvasAgg.copy_from_bbox` and
            `~FigureCanvasAgg.restore_region` methods.

        animated_artists : Iterable[Artist]
            List of the artists to manage
        """
        self.canvas = canvas
        self._bg = None
        self._artists = []

        for a in animated_artists:
            self.add_artist(a)
        # grab the background on every draw
        self.cid = canvas.mpl_connect("draw_event", self.on_draw)

    def on_draw(self, event):
        """Callback to register with 'draw_event'."""
        cv = self.canvas
        if event is not None:
            if event.canvas != cv:
                raise RuntimeError
        self._bg = cv.copy_from_bbox(cv.figure.bbox)
        self._draw_animated()

    def add_artist(self, art):
        """
        Add an artist to be managed.

        Parameters
        ----------
        art : Artist

            The artist to be added.  Will be set to 'animated' (just
            to be safe).  *art* must be in the figure associated with
            the canvas this class is managing.

        """
        if art.figure != self.canvas.figure:
            raise RuntimeError
        art.set_animated(True)
        self._artists.append(art)

    def _draw_animated(self):
        """Draw all of the animated artists."""
        fig = self.canvas.figure
        for a in self._artists:
            fig.draw_artist(a)

    def update(self):
        """Update the screen with animated artists."""
        cv = self.canvas
        fig = cv.figure
        # paranoia in case we missed the draw event,
        if self._bg is None:
            self.on_draw(None)
        else:
            # restore the background
            cv.restore_region(self._bg)
            # draw all of the animated artists
            self._draw_animated()
            # update the GUI state
            cv.blit(fig.bbox)
        # let the GUI event loop process anything it has to do
        cv.flush_events()
 
DAC_SAMPLE_RATE = 32_000
ADC_SAMPLE_RATE = 32_000
PLOT_SAMPLES = 12_000
PLOT_TIME = PLOT_SAMPLES / ADC_SAMPLE_RATE

SYMBOL_PERIOD = 5e-3
SAMPLES_PER_SYMBOL = int(SYMBOL_PERIOD * ADC_SAMPLE_RATE)


SAMPLE_BUF = np.array([0.0])
SAMPLE_IDX = 0

def refill_buf():
    global SAMPLE_BUF
    global FILT_POWER

    count = 2000
    rate = ADC_SAMPLE_RATE

    raw_count = count * DAC_SAMPLE_RATE // rate
    d = p.read(raw_count * 2)
    if p.in_waiting > count:
        print('FALLING BEHIND!')
    if d[1] > 16:
        print(f'Skipping byte! {d[0]} {d[1]} {d[2]}')
        d = d[1:] + p.read(1)
    samples = np.array([(int.from_bytes(d[i:i+2], 'little') - 2048.0) / 2048.0 for i in range(0, len(d), 2)])
    samples = signal.resample(samples, count)
    SAMPLE_BUF = np.concatenate([SAMPLE_BUF, samples])

FILT_POWER = 1.0

def get_samples(count, rate=ADC_SAMPLE_RATE, wait_until_start=True):
    global SAMPLE_BUF
    global SAMPLE_IDX
    global FILT_POWER

    while len(SAMPLE_BUF) < count:
        refill_buf()

    result = (SAMPLE_IDX, SAMPLE_BUF[:count])
    SAMPLE_BUF = SAMPLE_BUF[count:]
    SAMPLE_IDX += count
    return result


    skipped = 0
    while False and FILT_POWER > 1e-7:
        if len(SAMPLE_BUF) < 2:
            refill_buf()

        skipped += 1
        FILT_POWER = FILT_POWER * 0.99 + SAMPLE_BUF[0] * SAMPLE_BUF[0] * 0.01
        SAMPLE_BUF = SAMPLE_BUF[1:]
    
    print(skipped)

    while True:
        if len(SAMPLE_BUF) < 10:
            refill_buf()
        
        if abs(SAMPLE_BUF[9]) > 1e-2:
            break
         
        FILT_POWER = FILT_POWER * 0.99 + SAMPLE_BUF[0] * SAMPLE_BUF[0] * 0.01
        SAMPLE_BUF = SAMPLE_BUF[1:]
    
    while len(SAMPLE_BUF) < count:
        refill_buf()
    
    result = SAMPLE_BUF[:count]
    for i in range(count):
        FILT_POWER = FILT_POWER * 0.99 + SAMPLE_BUF[i] * SAMPLE_BUF[i] * 0.01
    SAMPLE_BUF = SAMPLE_BUF[count:]
    return result
    
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

#p = Serial('/dev/tty.usbmodem1401', 115200)
p = Serial('/dev/tty.usbmodem1401', 115200)

t = time.perf_counter()


plot_samples = np.zeros(PLOT_SAMPLES)

POWER_WINDOW_SIZE = WINDOW_SIZE * 2

SAMPLE_OFFSET = 40

pow_data = np.zeros(PLOT_SAMPLES // POWER_WINDOW_SIZE)

abc = 0

l = 0
while True:
    k = 0
    while True:
        t, samples = get_samples(POWER_WINDOW_SIZE)
        x = np.linspace(t, t + POWER_WINDOW_SIZE, POWER_WINDOW_SIZE, endpoint=False) / ADC_SAMPLE_RATE
        k += 1

        avg_power = 0.0
        for freq in CARRIER_FREQS:
            avg_power += np.average(np.abs(np.exp(1j * freq * 2.0 * math.pi * x) * samples))

        if True:
            if avg_power < 1.0:
            #if avg_power < 0.09:
                print('found quiet')
                break
        else:
            pow_data[l] = avg_power
            l += 1
            if l == PLOT_SAMPLES // POWER_WINDOW_SIZE:
                l = 0
                ln_power.set_ydata(np.repeat(pow_data, POWER_WINDOW_SIZE, axis=0))
                bm3.update()

    filt_power = 0.0

    t, samples = get_samples(PLOT_SAMPLES)
    x = np.linspace(t, t + PLOT_SAMPLES, PLOT_SAMPLES, endpoint=False) / ADC_SAMPLE_RATE
    i = WINDOW_SIZE
    while True:
        total_power = 0.0
        f = fft.fft(samples[i:i+64])
        for freq in CARRIER_FREQS:
            total_power += np.abs(f[round(freq / 500)])
        if total_power > 4.0:
            break
        i += 1
    
    i += SAMPLES_PER_SYMBOL * 2 // 8
    #i += 2
    t += i
    _, samples2 = get_samples(i)
    samples = np.concatenate([samples[i:], samples2])

    print(decode(samples, CARRIER_FREQS, ADC_SAMPLE_RATE, SYMBOL_PERIOD, skip=2, sample_offset=SAMPLE_OFFSET))

    sample_times = symbol_sample_indices(len(samples), SYMBOL_PERIOD, ADC_SAMPLE_RATE, sample_offset=SAMPLE_OFFSET) / ADC_SAMPLE_RATE

    print('SAMPLE TIMES: ', sample_times)

    iq_values = mix_and_filt(samples, CARRIER_FREQS, ADC_SAMPLE_RATE)

    #phase = np.angle(iq_lpf[SAMPLES_PER_SYMBOL * 3 // 2])
    #mag = np.abs(iq_lpf[SAMPLES_PER_SYMBOL * 3 // 2])
    #print(phase, mag)

    #iq_lpf = iq_lpf * (np.exp(-1j * phase) * 4.0 / mag)

    adjusts = 4.0 / iq_values[:,int(SAMPLES_PER_SYMBOL*2-SAMPLE_OFFSET)]
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
    #ln_p.set_ydata(np.log(pow_lpf) + 5.0)
    fr_number.set_text(f"time: {t / ADC_SAMPLE_RATE}")
    # tell the blitting manager to do its thing
    bm1.update()
    bm2.update()

    t3 = time.perf_counter()