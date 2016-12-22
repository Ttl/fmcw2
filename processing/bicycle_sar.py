import sys
import matplotlib.pyplot as plt
import matplotlib.image as image
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import struct
from scipy.signal import decimate
from scipy import interpolate as interp
import pickle

start = 2001
decimate_sweeps = 10
end = start+decimate_sweeps*1090
bit_depth = 2**10
decimate_samples = 8
bb_filter = None

bw = 200e6
sweep_length = 1.0e-3
sample_rate = 10.2e6/20

speed = 1.41

sweep_samples = sweep_length*sample_rate

header_length = 0

def read_bb_filter(filename):
    freqs, dbs, spec = [], [], []
    with open(filename, 'r') as f:
        for e,line in enumerate(f):
            if e == 0:
                #Skip header
                continue
            line = line.split('\t')
            freqs.append(float(line[0]))
            db, phase = line[1][1:-4].split(',')
            dbs.append(float(db[:-2]))
            spec.append(float(phase))
    return freqs, dbs, spec

if 1:
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

    print max(syncs)
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

if bb_filter:
    bb_freq, bb_db, bb_phase = read_bb_filter(bb_filter)
    max_f_i = np.argmax(bb_db)
    max_f = bb_freq[max_f_i]
    max_db = bb_db[max_f_i]
    db_end_i = max_f_i
    while bb_db[db_end_i] > max_db - 6.:
        db_end_i += 1
    db40_line = [ 40*np.log10(f) for f in bb_freq ]
    c = min([db40_line[i] - bb_db[i] for i in xrange(len(bb_db))])
    db40_line = [i-c for i in db40_line]
    db_corr = [db40_line[i] - bb_db[i] for i in xrange(db_end_i)]
    db_corri = interp.interp1d(bb_freq[:db_end_i], db_corr, fill_value=1, bounds_error=False)
    bb_p = interp.interp1d(bb_freq, bb_phase, fill_value=0, bounds_error=False)
    freqs = lambda i: i*sample_rate/(2.0*len(sweeps[0]))
    correction = []
    db_correction = []
    for i in xrange(len(sweeps[0])//2):
        phase = bb_p(freqs(i))
        phase = -phase
        phase *= 0.0174532925 #To radians
        correction.append(np.exp(1j*phase))
        if freqs(i) < bb_freq[db_end_i]:
            db_correction.append(10**(db_corri(freqs(i))/20.))
        else:
            db_correction.append(10**(db_corri(freqs(bb_freq[db_end_i]))/20.))
    #Hilbert transform and combined phase correction
    for e in xrange(len(sweeps)):
        fx = np.fft.fft(sweeps[e])
        fx[len(fx)/2+1:] = 0
        fx[0] = 0 # Zero DC component
        fx[:len(correction)] *= correction
        fx[:len(db_correction)] *= db_correction
        fx[len(sweeps[e])/decimate_samples:] *= 0
        #FIXME: Decimate can't handle complex data, so undo Hilbert transform
        sweeps[e] = np.real(np.fft.ifft(fx))

sweeps = list(sweeps)

if 1:
    sample_rate /= decimate_samples
    min_sync /= decimate_samples
    if decimate_samples > 1:
        sweeps = map(lambda x: decimate(x, decimate_samples, ftype='fir'), sweeps)
    delta_crange = speed*sweep_length*decimate_sweeps

    min_len = min(map(len, sweeps))
    print "Minimum length",min_len
    sweeps = [i[:min_len] for i in sweeps]

    max_range = 3e8*sample_rate*sweep_length/(4*bw)
    print max_range

    if 0:
        with open('sar_data.p', 'w') as f:
            pickle.dump( (f0, bw, sweep_length, sweeps, 0, max_range, delta_crange), f )

if 1:
    max_range = 180
    subtract_clutter = False
    subtract_background = False

    if subtract_background:
        averages = 0
        background = [0]*min_sync
        for sw in sweeps:
            if len(sw) != min_sync:
                continue
            background = [background[i]+sw[i] for i in xrange(min_sync)]
            averages += 1
        if averages > 0:
            background = [i/averages for i in background]

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
                if len(sw) != len(swp):
                    #print e,len(sw),len(swp)
                    continue
                sw = [sw[n]-swp[n] for n in xrange(len(sw))]
        if subtract_background:
            if len(sw) != len(background):
                continue
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
            #np.linspace(0,decimate_sweeps*2*lines*sw_len/sample_rate, im.shape[1]),
            np.linspace(0, im.shape[1], im.shape[1]),
            np.linspace(0, 3e8*max_range_index*sample_rate/(2*sw_len)/((bw/sweep_length)), im.shape[0]))

        imgplot = plt.pcolormesh(xx,yy,im)
        imgplot.set_clim(m-100,m)

        plt.colorbar()
        image.imsave('range_time_raw.png', np.flipud(im))
        plt.savefig('range_time.png', dpi=500)
        plt.show()

