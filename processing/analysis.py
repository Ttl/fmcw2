import sys
import matplotlib.pyplot as plt
import matplotlib.image as image
import numpy as np
import struct

#Read sweeps from start to end
start = 200
end = 1000000
#Only takes every nth sweep. 1 to process all sweeps.
decimate_sweeps = 1

bit_depth = 2**10.
adc_ref = 1.

#Default values if not written in the header
bw = 200e6
sweep_length = 1.0e-3
sample_rate = 10.2e6/20

sweep_samples = sweep_length*sample_rate

header_length = 0
filename = sys.argv[1]

#Read header
with open(filename, 'r') as f:
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
        if version > 0:
            sweep_delay = struct.unpack('<L',f.read(4))[0]
            flags = struct.unpack('<L',f.read(4))[0]
            sweep_delay = int((sweep_delay/30e6)*sample_rate)
        else:
            flags = struct.unpack('<L',f.read(4))[0]
            sweep_delay = 0
        print sample_rate, f0, bw, sweep_length, sweep_delay
    else:
        print "Invalid header"

with open(filename+'.sync', 'r') as f:
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
with open(filename, 'r') as f:
    print 'Drop samples',drop_samples
    f.seek(drop_samples+header_length)
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
        sweeps.append(samples[sweep_delay:])

#Long FFT over multiple sweeps
if 0:
    def moving_average(a, n=10) :
        ret = np.cumsum(a, dtype=float)
        ret[n:] = ret[n:] - ret[:-n]
        return ret[n - 1:] / n

    y = []
    for i in xrange(10):
        y.extend(sweeps[i])

    w = np.hanning(len(y))
    y = [y[i]*w[i] for i in xrange(len(w))]

    fy = (adc_ref/(bit_depth*len(y)))*np.fft.rfft(y)
    plt.figure()
    plt.title(filename+': Long FFT')
    averages = 10
    fy = 20*np.log10(moving_average(np.abs(fy), averages))
    fx = [i*sample_rate/(2*len(fy)) for i in xrange(len(fy))]
    plt.plot(fx[1:],fy[1:])
    plt.show()

#FFT of averaged sweeps
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
    plt.figure()
    plt.title(filename+': Averaged sweep')
    plt.plot(fx[1:],20*np.log10((adc_ref/(bit_depth*len(y)))*abs(fy)[1:]))
    plt.show()

#View time domain waveform
if 0:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtGui, QtCore
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

#Time domain plot of multiple sweeps with matplotlib
if 0:
    plt.figure()
    plt.title(filename+': Multiple sweeps')
    for i in xrange(10,15):
        plt.plot(sweeps[i])
    plt.show()

#Single sweep FFT and time domain
if 0:
    y = []
    y = sweeps[min(120,len(sweeps)-1)]
    w = np.hanning(len(y))
    plt.figure()
    plt.title(filename+': Single sweep')
    plt.plot(y)
    y = [y[i]*w[i] for i in xrange(len(w))]

    fy = np.fft.rfft(y)
    fx = [i*sample_rate/(2*len(fy)) for i in xrange(len(fy))]
    fx = [3e8*i/(2*(bw/sweep_length)) for i in fx]
    plt.figure()
    plt.title(filename+': Single sweep FFT')
    plt.xlabel("Distance [m]")
    plt.plot(fx[1:],20*np.log10((adc_ref/(bit_depth*len(y)))*abs(fy)[1:]))
    plt.show()

if 1:
    #These can be modified
    max_range = 200
    subtract_clutter = False
    subtract_background = False

    sw_len = min_sync - sweep_delay

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
        sweeps[e] = sweeps[e][:sw_len]
        sweeps[e].extend([0]*(sw_len-len(sweeps[e])))

    lines = len(sweeps)
    print lines, "lines"
    fourier_len = len(sweeps[0])/2
    max_range_index = int((4*bw*fourier_len*max_range)/(3e8*sample_rate*sweep_length))
    max_range_index = min(max_range_index, sw_len//2)
    print max_range_index
    im = np.zeros((max_range_index-2, lines))
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
        fy = np.fft.rfft(sw)[3:max_range_index+1]
        fy = 20*np.log10((adc_ref/(bit_depth*max_range_index))*np.abs(fy))
        fy = np.clip(fy, -100, float('inf'))
        m = max(m,max(fy))
        im[:,e] = np.array(fy)

    if 1:
        f = sample_rate/2.0

        if 0:
            xx, yy = np.meshgrid(
                np.linspace(0,decimate_sweeps*2*lines*sw_len/sample_rate, im.shape[1]),
                np.linspace(0, 3e8*max_range_index*sample_rate/(2*sw_len)/((bw/sweep_length)), im.shape[0]))
        else:
            xx, yy = np.meshgrid(
                np.linspace(0,im.shape[1]-1, im.shape[1]),
                np.linspace(0, 3e8*max_range_index*sample_rate/(2*sw_len)/((bw/sweep_length)), im.shape[0]))
        plt.ylabel("Range [m]")
        plt.xlabel("Time [s]")
        plt.title(filename+' Range-time plot')
        imgplot = plt.pcolormesh(xx,yy,im)
        imgplot.set_clim(m-100,m)
        plt.colorbar()
        #Save png of the plot
        image.imsave('range_time_raw.png', np.flipud(im))
        plt.savefig('range_time.png', dpi=500)
        plt.show()
