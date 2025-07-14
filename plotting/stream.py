from plotutil import *
from codec import *
from queue import Queue

SEARCH_SAMPLES = 48_000
PLOT_SAMPLES = 48_000

p = WaveSource('/dev/tty.usbmodem1203', in_rate=ADC_SAMPLE_RATE, out_rate=ADC_SAMPLE_RATE)

segment_queue = Queue()

def streaming_mix_filt(sample_idx, sig, freqs, sample_rate):
    samples = sig.shape[0]

    times = np.linspace(sample_idx/sample_rate, (sample_idx+samples)/sample_rate, num=len(sig), endpoint=False)
    mixed = sig.reshape((1, samples)) * np.exp(2.0j * math.pi * np.outer(freqs, times))

    iq_filt = np.zeros_like(mixed)
    window_filt = np.concatenate([np.zeros(WINDOW_SIZE-1), np.ones(WINDOW_SIZE)]) / WINDOW_SIZE
    for f in range(len(freqs)):
        iq_filt[f,:] = np.convolve(mixed[f,:], window_filt, mode='same')
    
    return iq_filt

def get_segments():
    samples = np.array([0.0])

    def refill_buffer():
        nonlocal samples
        while len(samples) < SEARCH_SAMPLES:
            _, new_samples = p.get_samples(SEARCH_SAMPLES)
            samples = np.concatenate([samples, new_samples], axis=0)

    while True:
        while len(samples) < PLOT_SAMPLES:
            refill_buffer()
        segment_queue.put(samples[:PLOT_SAMPLES])
        samples = samples[PLOT_SAMPLES:]

segment_thread = Thread(target=get_segments)
segment_thread.start()

# make a new figure
fig1, (ax_sig, ax_pow) = plt.subplots(nrows=2, sharex=True, num=1)

x = np.linspace(0, PLOT_SAMPLES / ADC_SAMPLE_RATE, num=PLOT_SAMPLES, endpoint=False)

# add a line
ax_sig.set_ylim(bottom=-32768, top=32768)
(ln_sig,) = ax_sig.plot(x, np.zeros_like(x), animated=True)

(ln_pow,) = ax_pow.plot(x, np.zeros_like(x), animated=True)

bm1 = BlitManager(fig1.canvas, [ln_sig, ln_pow])

plt.show(block=False)
plt.pause(0.1)

plot_samples = segment_queue.get()

while True:
    while True:
        try:
            plot_samples = segment_queue.get_nowait()
        except queue.Empty:
            break

    print(plot_samples[:10])
    ln_sig.set_ydata(plot_samples)
    bm1.update()