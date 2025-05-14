from serial import Serial
import matplotlib.pyplot as plt
import numpy as np
import time
import math
from scipy import signal

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

DAC_SAMPLE_RATE = 100_000
ADC_SAMPLE_RATE = 32_000
PLOT_SAMPLES = 16_000
PLOT_TIME = PLOT_SAMPLES / ADC_SAMPLE_RATE
WINDOW_SIZE = 64

SYMBOL_PERIOD = 5e-3
SAMPLES_PER_SYMBOL = int(SYMBOL_PERIOD * ADC_SAMPLE_RATE)


SAMPLE_BUF = np.array([0.0])

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
    global FILT_POWER

    skipped = 0
    while FILT_POWER > 1e-7:
        if len(SAMPLE_BUF) == 0:
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
ax_cons.set_xlim(left=-5, right=5)
ax_cons.set_ylim(bottom=-5, top=5)
ax_cons.scatter(i_correct, q_correct)
ln_cons = ax_cons.scatter(i_correct, q_correct, animated=True)

bm1 = BlitManager(fig1.canvas, [ln_sig, fr_number, ln_i, ln_q, ln_p])
bm2 = BlitManager(fig2.canvas, [ln_cons])
# make sure our window is on the screen and drawn
plt.show(block=False)
plt.pause(.1)

p = Serial('/dev/tty.usbmodem1301', 115200)

t = time.perf_counter()

freq = 5.5e3

j = 0
while True:
    samples = get_samples(PLOT_SAMPLES)

    x = np.linspace(PLOT_TIME * j, PLOT_TIME * (j + 1), PLOT_SAMPLES, endpoint=False)

    cos_part = np.cos(freq * 2.0 * math.pi * x)
    sin_part = np.sin(freq * 2.0 * math.pi * x)

    cos_mixed = cos_part * samples
    sin_mixed = sin_part * samples

    window_filt = np.concatenate([np.zeros(WINDOW_SIZE-1), np.ones(WINDOW_SIZE)]) / WINDOW_SIZE

    iq_lpf = np.convolve(cos_mixed, window_filt, mode='same') + 1j * np.convolve(sin_mixed, window_filt, mode='same')

    phase = np.angle(iq_lpf[SAMPLES_PER_SYMBOL * 3 // 2])
    mag = np.abs(iq_lpf[SAMPLES_PER_SYMBOL * 3 // 2])
    print(phase, mag)

    iq_lpf = iq_lpf * (np.exp(-1j * phase) * 4.0 / mag)

    cons_points = iq_lpf[SAMPLES_PER_SYMBOL::SAMPLES_PER_SYMBOL].copy()

    j += 1

    # update the artists
    #ln.set_xdata(x)
    ln_sig.set_ydata(samples)
    ln_i.set_ydata(np.real(iq_lpf))
    ln_q.set_ydata(np.imag(iq_lpf))
    # From https://stackoverflow.com/questions/38900344/convert-complex-numpy-array-into-n-2-array-of-real-and-imaginary-parts
    ln_cons.set_offsets(cons_points.view(float).reshape(-1, 2))
    #ln_p.set_ydata(np.log(pow_lpf) + 5.0)
    fr_number.set_text(f"frame: {j}")
    # tell the blitting manager to do its thing
    bm1.update()
    bm2.update()

    t3 = time.perf_counter()