import sys
import matplotlib.pyplot as plt
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import struct

start = 0e6
end = start+20e6

start = int(start)
end = int(end)

#Must be aligned to 2 bytes
start = start - start%2
end = end - end%2

bw = 200e6
sweep_length = 1.0e-3
sample_rate = 10.2e6/20

sweep_samples = sweep_length*sample_rate
print sweep_samples

header_length = 0

#Read header
with open(sys.argv[1], 'r') as f:
    magic = f.read(4)
    if magic == "FMCW":
        version = struct.unpack('<L',f.read(4))[0]
        header_length = struct.unpack('<L',f.read(4))[0]
        sample_rate = struct.unpack('<d',f.read(8))[0]
        f0 = struct.unpack('<d',f.read(8))[0]
        bw = struct.unpack('<d',f.read(8))[0]
        sweep_length = struct.unpack('<d',f.read(8))[0]
        flags = struct.unpack('<L',f.read(4))[0]
        print sample_rate, f0, bw, sweep_length
    else:
        print "Invalid header"

with open(sys.argv[1], 'r') as f:
    f.seek(start+header_length)
    samples_in = f.read(end-start)
    samples = []
    for i in xrange(0,len(samples_in),2):
        d = samples_in[i:i+2]
        samples.append(struct.unpack('<h',d)[0]/2**10.)

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

if 0:
    av = 0
    for i in xrange(len(samples)/2,len(samples)):
        av += samples[i]
    print (2.0*av)/len(samples)

#Legacy
#Add missed syncs

#First find the minimum period
#Filter for too short periods
min_sync = min([s for s in syncs[:10] if s > sweep_samples/3])

if 0:
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
        sw = samples[start:start+syncs[i]]
        if len(sw) < syncs[i]:
            #No more data
            break
        sweeps.append(sw)
        start += syncs[i]
        i += 1
    except IndexError:
        break

print len(sweeps)

if 1:
    sweep_average = [s for s in sweeps if len(s) == min_sync]
    sweep_average = zip(*sweep_average)
    sweep_average = [sum(i)/len(i) for i in sweep_average]

    y = sweep_average
    w = np.hanning(len(y))
    plt.figure()
    plt.plot(y)
    y = [y[i]*w[i] for i in xrange(len(w))]

    fy = np.fft.rfft(y)
    fx = [i*sample_rate/(2*len(fy)) for i in xrange(len(fy))]
    #fx = [3e8*i/(2*(500e6/1.0e-3)) for i in fx]
    plt.figure()
    plt.plot(fx[1:],20*np.log(abs(fy)[1:]))
    plt.show()

if 0:
    app = QtGui.QApplication([])
    win = pg.GraphicsWindow()
    p1 = win.addPlot(title="Samples")
    p1.plot(samples)
    if __name__ == '__main__':
        import sys
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()

if 0:
    plt.figure()
    for i in xrange(50,100,2):
        plt.plot(sweeps[i])
        print len(sweeps[i])
    plt.show()

if 1:
    y = sweeps[100]
    y = [y[i] - sweeps[98][i] for i in xrange(len(y))]
    y2 = sweeps[100]
    w = np.hanning(len(y))
    plt.figure()
    plt.plot(y)
    plt.plot(y2)
    y = [y[i]*w[i] for i in xrange(len(w))]
    y2 = [y2[i]*w[i] for i in xrange(len(w))]

    fy = np.fft.rfft(y)
    fy2 = np.fft.rfft(y2)
    fx = [i*sample_rate/(2*len(fy)) for i in xrange(len(fy))]
    #fx = [3e8*i/(2*(500e6/1.0e-3)) for i in fx]
    plt.figure()
    plt.plot(fx[1:],20*np.log(abs(fy)[1:]))
    plt.plot(fx[1:],20*np.log(abs(fy2)[1:]))
    plt.show()



lens = sorted([len(sw) for sw in sweeps])
print len(sweeps)

if 1:
    lines = len(sweeps)/4
    print lines, "lines"
    sw_len = len(sweeps[0])
    fourier_len = len(sweeps[0])/2
    im = np.zeros((fourier_len, lines))
    w = np.hanning(sw_len)
    fx = [i*sample_rate/(2*fourier_len) for i in xrange(fourier_len)]
    for e in xrange(0,len(sweeps),2):
        sw = sweeps[e][:sw_len]
        swp = sweeps[e-2][:sw_len]
        if len(sw) != len(swp):
            print e,len(sw),len(swp)
            continue
        sw = [sw[n]-swp[n] for n in xrange(len(sw))]
        if e >= lines:
            break
        if len(sw) < len(w):
            print e,len(sw)
            continue
        sw = [sw[i]*w[i] for i in xrange(len(w))]
        fy = np.fft.rfft(sw)[1:]
        fy = 20*np.log(abs(fy))
        im[:,e/2] = np.array(fy)

    if 1:
        f = sample_rate/2.0
        print f
        print 3e8*f/(2*(bw/sweep_length))
        xx, yy = np.meshgrid(
            np.linspace(0,2*lines*sw_len/sample_rate, im.shape[1]),
            np.linspace(0, 3e8*f/(2*(bw/sweep_length)), im.shape[0]))

        plt.pcolormesh(xx,yy,im)
        plt.colorbar()
        plt.show()

    if 0:
        app = QtGui.QApplication([])
        pg.image(im)
        if __name__ == '__main__':
            import sys
            if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
                QtGui.QApplication.instance().exec_()
#plt.specgram(samples, NFFT=2*2048, Fs=sample_rate, noverlap=10)
#plt.show()
