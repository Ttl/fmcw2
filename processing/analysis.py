import sys
import matplotlib.pyplot as plt
import matplotlib.image as image
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import struct

start = 200
end = 200000
decimate_sweeps = 100
bit_depth = 2**10

bw = 200e6
sweep_length = 1.0e-3
sample_rate = 10.2e6/20

sweep_samples = sweep_length*sample_rate

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
        sweep_samples = sweep_length*sample_rate
        print sample_rate, f0, bw, sweep_length
    else:
        print "Invalid header"

with open(sys.argv[1]+'.sync', 'r') as f:
    samples_in = f.read()
    syncs = []
    for i in xrange(0,len(samples_in),4):
        d = samples_in[i:i+4]
        syncs.append(struct.unpack('<L',d)[0])

print len(syncs),"syncs"
drop_samples = 2*sum(syncs[:start])
syncs = syncs[start:end]

def most_common(lst):
    return max(set(lst), key=lst.count)

#First find the minimum period
#Filter for too short periods
min_sync = most_common([s for s in syncs if s > sweep_samples/2.0])

#Fix noise in syncs
for e,s in enumerate(syncs):
    if s < min_sync - 5:
        acc = s
        i = e
        while acc < min_sync - 5:
            acc += syncs[i]
            i += 1
        if abs(acc - min_sync) < 5:
            syncs[e] = acc
            for j in xrange(e,i):
                del syncs[j]

#Read samples
sweeps = []
with open(sys.argv[1], 'r') as f:
    print 'Drop samples',drop_samples
    f.seek(drop_samples+header_length)
    #samples_in = f.read(2*sum(syncs))
    for s in xrange(end-start):
        try:
            samples_in = f.read(2*syncs[s])
        except IndexError:
            break
        if s % decimate_sweeps != 0:
            continue
        samples = []
        for i in xrange(0,len(samples_in),2):
            d = samples_in[i:i+2]
            samples.append(struct.unpack('<h',d)[0])
        sweeps.append(samples)

if 0:
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
    samples = sweeps[len(sweeps)/2:len(sweeps)/2+10]
    samples = [item for sublist in samples for item in sublist]
    p1.plot(samples)
    if __name__ == '__main__':
        import sys
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()

if 0:
    plt.figure()
    for i in xrange(100,104):
        plt.plot(sweeps[i])
    plt.show()

if 0:
    y = sweeps[100]
    w = np.hanning(len(y))
    plt.figure()
    plt.plot(y)
    y = [y[i]*w[i] for i in xrange(len(w))]

    fy = np.fft.rfft(y)
    fx = [i*sample_rate/(2*len(fy)) for i in xrange(len(fy))]
    fx = [3e8*i/(2*(bw/sweep_length)) for i in fx]
    plt.figure()
    plt.xlabel("Distance [m]")
    plt.plot(fx[1:],20*np.log(abs(fy)[1:]))
    plt.show()

if 1:
    max_range = 150
    subtract_clutter = False
    subtract_background = False

    if subtract_background:
        averages = 0
        background = [0]*min_sync
        for sw in sweeps:
            if len(sw) < min_sync:
                continue
            background = [background[i]+sw[i] for i in xrange(min_sync)]
            averages += 1
        if averages > 0:
            background = [i/averages for i in background]

    for e in xrange(len(sweeps)):
        sweeps[e] = sweeps[e][:min_sync]
        sweeps[e].extend([0]*(min_sync-len(sweeps[e])))

    lines = len(sweeps)
    print lines, "lines"
    sw_len = min_sync
    fourier_len = len(sweeps[0])/2
    max_range_index = int((4*bw*fourier_len*max_range)/(3e8*sample_rate*sweep_length))
    print max_range_index
    im = np.zeros((max_range_index, lines))
    w = np.hanning(sw_len)
    m = 0


    for e in xrange(0,len(sweeps)):
        sw = sweeps[e][:sw_len]
        if subtract_clutter:
            if e > 1:
                swp = sweeps[e-1][:sw_len]
                #if len(sw) != len(swp):
                #    #print e,len(sw),len(swp)
                #    continue
                sw = [sw[n]-swp[n] for n in xrange(min(len(swp),len(sw)))]
        if subtract_background:
            sw = [sw[n]-background[n] for n in xrange(len(sw))]
        if e >= lines:
            break
        if len(sw) < len(w):
            if (len(w) - len(sw)) < 3:
                sw.extend([0]*(len(w)-len(sw)))
            else:
                #print "Short sweep",e,len(w),len(sw)
                continue
        sw = [sw[i]*w[i] for i in xrange(len(w))]
        fy = np.fft.rfft(sw)[1:max_range_index+1]
        fy = fy/bit_depth
        fy = 20*np.log(abs(fy))
        fy = np.clip(fy, -40, float('inf'))
        m = max(m,max(fy))
        im[:,e] = np.array(fy)

    if 1:
        f = sample_rate/2.0
        xx, yy = np.meshgrid(
            np.linspace(0,decimate_sweeps*2*lines*sw_len/sample_rate, im.shape[1]),
            #np.linspace(0, im.shape[1], im.shape[1]),
            np.linspace(0, 3e8*max_range_index*sample_rate/(2*sw_len)/((bw/sweep_length)), im.shape[0]))

        plt.ylabel("Range [m]")
        plt.xlabel("Time [s]")
        imgplot = plt.pcolormesh(xx,yy,im)
        imgplot.set_clim(m-100,m)
        plt.colorbar()
        image.imsave('range_time_raw.png', np.flipud(im))
        plt.savefig('range_time.png', dpi=500)
        plt.show()

    if 0:
        app = QtGui.QApplication([])
        pg.image(im)
        if __name__ == '__main__':
            import sys
            if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
                QtGui.QApplication.instance().exec_()
