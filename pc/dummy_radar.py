import numpy as np
import time

class Dummy_radar():
    def __init__(self):
        self.rf_power = False
        self.fstep = 0
        self.fstart = 5.5e9
        self.fstop = 6.0e9
        self.bb_gain = 128
        self.trig = None
        self.debug_enable = True

        self.bb_srate = 10e6
        self.sweep_length = 0.5e-3

    def debug_print(self, s):
        if self.debug_enable:
            print s

    def set_pa(self, on):
        self.debug_print("set_pa {}".format(on))
        self.rf_power = on

    def set_trig(self, trig):
        self.debug_print("set_trig {}".format(trig))
        self.trig = trig

    def set_sweep(self, start, stop, length):
        self.fstart = float(start)
        self.fstop = float(stop)
        self.sweep_length = float(length)
        self.debug_print("set_sweep {:.4} {:.4} {:.4}".format(start, stop, length))

    def stream(self):
        f = 1000
        t = np.linspace(0, self.sweep_length, self.bb_srate*self.sweep_length)
        while True:
            tstart = time.time()
            yield np.sin(2*np.pi*f*t)
            time.sleep(0.01)
            #print time.time() - tstart
            f += 100.
            if f > self.bb_srate/2.:
                f = 1000
