import numpy as np
import time
from threading import Thread, Lock
import queue
from queue import Queue
from serial import Serial
from scipy import signal
from codec import *

DAC_SAMPLE_RATE = 100_000
ADC_SAMPLE_RATE = 32_000
SYMBOL_PERIOD = 4e-3

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
 
class WaveSource:
    def __init__(self, port, in_rate, out_rate):
        self.port = Serial(port, 115200)
        self.in_rate = in_rate
        self.out_rate = out_rate
        self.sample_buf = np.array([0.0])
        self.sample_idx = 0
        self.sample_queue = Queue()
        self.read_thread = Thread(target=self._read_port)
        self.read_thread.start()
    
    def _read_port(self):
        count = 2000
        block_size = count * self.in_rate // self.out_rate
        while True:
            d = self.port.read(block_size * 2)
            #if d[1] > 16:
            #    print(f'Skipping byte! {d[0]} {d[1]} {d[2]}')
            #    d = d[1:] + self.port.read(1)
            samples = np.array([(int.from_bytes(d[i:i+2], 'little', signed=True)) for i in range(0, len(d), 2)])
            samples = signal.resample(samples, count)
            self.sample_queue.put(samples)

    def refill_buf(self, callback=None):
        count = 2000

        s = time.perf_counter()

        raw_count = count * self.in_rate // self.out_rate
        d = b''
        while len(d) < raw_count * 2:
            l = min(raw_count * 2 - len(d), self.port.in_waiting)
            d += self.port.read(l)
            if callback is not None:
                callback()
        if self.port.in_waiting > count:
            print('FALLING BEHIND!')

        print(raw_count / (time.perf_counter() - s), ' samples per s')

    
    def get_samples(self, count, callback=None):
        while len(self.sample_buf) < count:
            try:
                while True:
                    new_samples = self.sample_queue.get_nowait()
                    self.sample_buf = np.concatenate([self.sample_buf, new_samples])
            except queue.Empty:
                pass

        result = (self.sample_idx, self.sample_buf[:count])
        self.sample_buf = self.sample_buf[count:]
        self.sample_idx += count
        return result