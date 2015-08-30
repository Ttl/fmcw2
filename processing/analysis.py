import sys
import matplotlib.pyplot as plt
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import struct

start = 10e6
end = start+10e6

start = int(start)
end = int(end)

#Must be aligned to 2 bytes
start = start - start%2
end = end - end%2

sweep = 0.5e-3
sample_rate = 10.2e6/5

sweep_samples = sweep*sample_rate*2
print sweep_samples

with open(sys.argv[1], 'r') as f:
    f.seek(start)
    samples_in = f.read(end-start)
    samples = []
    for i in xrange(0,len(samples_in),2):
        d = samples_in[i:i+2]
        samples.append(struct.unpack('<h',d)[0])

with open(sys.argv[1]+'.sync', 'r') as f:
    samples_in = f.read()
    syncs = []
    for i in xrange(0,len(samples_in),4):
        d = samples_in[i:i+4]
        syncs.append(struct.unpack('<L',d)[0])

sweep_start = 0
i = 0
while sweep_start < start/2:
    sweep_start += syncs[i]
    i += 1

start_index = sweep_start - start/2
samples = samples[start_index:]
syncs = syncs[i:]

#Add missed syncs

#First find the minimum period
#Filter for too short periods
min_sync = min([s for s in syncs[:50] if s > sweep_samples/4])

fixed_syncs = []
for s in syncs:
    if float(s)/min_sync > 1.5:
        a = [min_sync]*((s-min_sync)/min_sync)
        a.append(s-(min_sync)*((s/min_sync)-1))
        assert sum(a) == s
        fixed_syncs.extend(a)

syncs = fixed_syncs
sweeps = []
i = 0
start = 0
while True:
    try:
        sweeps.append(samples[start:start+syncs[i]])
        start += syncs[i]
        i += 1
    except IndexError:
        break

if 1:
    app = QtGui.QApplication([])
    win = pg.GraphicsWindow()
    p1 = win.addPlot(title="Samples")
    p1.plot(samples)
    if __name__ == '__main__':
        import sys
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()

if 1:
    plt.figure()
    for i in xrange(50,100,2):
        plt.plot(sweeps[i])
    plt.show()

if 1:
    y = sweeps[10]
    w = np.hanning(len(y))
    plt.figure()
    plt.plot(y)
    y = [y[i]*w[i] for i in xrange(len(w))]

    fy = np.fft.rfft(y)
    fx = [i*sample_rate/(2*len(fy)) for i in xrange(len(fy))]
    fx = [3e8*i/(2*(500e6/1.0e-3)) for i in fx]
    plt.figure()
    plt.plot(fx[1:],20*np.log(abs(fy)[1:]))
    plt.show()

#plt.specgram(samples, NFFT=2*2048, Fs=sample_rate, noverlap=10)
#plt.show()
