import scipy.signal
import numpy as np
import matplotlib.pyplot as plt

N = 75
nyquist = 5.1e6/2.
cutoff = 200e3

def conv(taps, signal):
    out = []
    for i in xrange(len(signal)-len(taps)):
        x = 0
        for j in xrange(len(taps)):
            x += signal[i+j] * taps[-j]
        out.append(x)
    return out

signal = [np.sin(2*np.pi*1e6*t/(2*nyquist)) + np.sin(2*np.pi*3e6*t/(2*nyquist)) for t in xrange(200)]
taps = scipy.signal.firwin(N, cutoff, nyq=nyquist)
for tap in taps:
    print "{},".format(tap)
w, h = scipy.signal.freqz(taps, worN=8000)
plt.figure()
plt.plot((w/np.pi)*nyquist, 20*np.log10(np.abs(h)), linewidth=2)
plt.figure()
plt.plot(signal)
plt.plot(conv(taps,signal))
plt.figure()
plt.plot(20*np.log10(np.abs(np.fft.rfft(conv(taps,signal)))))
plt.plot(20*np.log10(np.abs(np.fft.rfft(signal))))
plt.show()
