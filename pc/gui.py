import numpy as np
import sys
from PySide.QtGui import QApplication, QMainWindow, QVBoxLayout
from PySide import QtCore
import dummy_radar
import time
import connect_radar

from fmcw_gui import Ui_MainWindow

import matplotlib
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from multiprocessing import Process, Queue

import PySide
import pyqtgraph as pg

matplotlib.use('Qt4Agg')
matplotlib.rcParams['backend.qt4']='PySide'

pg.setConfigOptions(antialias=True)

use_multiprocessing = False

class MatplotlibWidget(FigureCanvas):
    def __init__(self, parent=None,xlabel='x',ylabel='y'):
        super(MatplotlibWidget, self).__init__(Figure())

        self.setParent(parent)
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        self.axes = self.figure.add_subplot(111)

        self.axes.set_xlabel(xlabel)
        self.axes.set_ylabel(ylabel)

def radar_streamer(sample_queue, radar):
    """
    Stream samples from radar.
    Run in other process to make sure that drawing GUI doesn't make us miss any samples.
    """
    for sample in radar.stream():
        #TODO: Log sample
        if sample_queue.empty():
            sample_queue.put(sample)

class Runner(QtCore.QThread):
    """
    Runs a job in a separate process and forwards messages from the job to the
    main thread through a pyqtSignal.
    """

    message = QtCore.Signal(list)

    def __init__(self, radar, parent=None):
        super(Runner, self).__init__(parent)
        self.radar = radar
        self.p = None
        self.stopping = False

    def run(self):
        if use_multiprocessing:
            queue = Queue()
            self.p = Process(target=radar_streamer, args=(queue, self.radar))
            #Closes the process when application closes
            self.p.daemon = True
            self.p.start()
            while True:
                msg = queue.get()
                self.message.emit(msg)
                #Time for updating the GUI
                #time.sleep(1/60.)
        else:
            for sample in self.radar.stream():
                self.message.emit(sample)
                if self.stopping:
                    break

    def stop(self):
        self.stopping = True
        if self.p:
            self.p.terminate()
        self.exit()

def fft(data, window=None):
    if window:
        w = window(len(data))
        data = [data[i]*w[i] for i in xrange(len(w))]
    y = map(abs,np.fft.rfft(data))
    return y

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        #Initialize variables
        self.threads = []
        self.radar = None
        self.status = 'Disconnected'
        super(MainWindow, self).__init__(parent)
        #Initialize UI
        self.setupUi(self)
        self.startSpinBox.valueChanged.connect(self.set_sweep)
        self.stopSpinBox.valueChanged.connect(self.set_sweep)
        self.lengthSpinBox.valueChanged.connect(self.set_sweep)

        #Menubar
        self.actionExit.setShortcut('Ctrl+Q')
        self.actionExit.setStatusTip('Exit application')
        self.actionExit.triggered.connect(self.close)

        self.actionDetect_radars.setShortcut('Ctrl+D')
        self.actionDetect_radars.setStatusTip('Detect connected radars')
        self.actionDetect_radars.triggered.connect(self.detect_radars)

        self.timeplot_type = None
        self.radioButton_range.toggled.connect(self.set_timeplot)
        self.radioButton_time.toggled.connect(self.set_timeplot)
        self.radioButton_none.toggled.connect(self.set_timeplot)

        self.radioButton_bigrange.toggled.connect(self.set_rangeplot)
        self.radioButton_bigwaterfall.toggled.connect(self.set_rangeplot)

        self.radioTrigSingle.toggled.connect(self.set_trig)
        self.radioTrigCont.toggled.connect(self.set_trig)
        self.pushButtonTrigger.clicked.connect(self.trigger_once)

        self.rfpowerBox.toggled.connect(self.set_rfpower)

        self.detect_radars()

        #Initialize time plot
        self.setupPlot()

        #Set the plot types
        self.set_timeplot()
        self.set_rangeplot()

        #Maximum number of samples in waterfall plot
        self.waterfall_history_length = 500

        self.print_status()

    def detect_radars(self):
        if self.radar:
            self.radar.disconnect()
            self.stopWorkers()
            self.radar = None
        radar_class = connect_radar.find_radar()
        if radar_class == None:
            return
        self.status = "Connected"
        self.print_status()
        self.radar = radar_class()
        print "Found radar {}".format(self.radar.name)
        self.radar.connect()
        #Make sure radar and GUI have same settings
        self.set_sweep()
        self.addWorker(Runner(self.radar))
        self.startWorkers()
        self.print_status()

    def print_status(self):
        try:
            name = self.radar.name
        except AttributeError:
            name = ""
        self.statusBar().showMessage("{} - {}".format(self.status,name))

    def addWorker(self, worker):
        worker.message.connect(self.plotTimePoints, QtCore.Qt.QueuedConnection)
        # connect the finished signal to method so that we are notified
        worker.finished.connect(self.workersFinished)
        self.threads.append(worker)

    def set_trig(self):
        trig = None
        if self.radioTrigSingle.isChecked():
            #Disable triggering
            trig = None
        elif self.radioTrigCont.isChecked():
            trig = 'cont'
        self.radar.set_trig(trig)

    def trigger_once(self):
        self.radar.set_trig('single')

    def set_rangeplot(self):
        """Change range plot configuration"""
        if self.radioButton_bigrange.isChecked():
            self.rangeplot_type = "range"
        if self.radioButton_bigwaterfall.isChecked():
            self.rangeplot_type = "waterfall"

    def set_timeplot(self):
        """Change time plot configuration"""
        self.timeplot_valid = False
        if self.radioButton_none.isChecked():
            self.timeplot_type = "none"
        if self.radioButton_range.isChecked():
            self.timeplot_type = "freq"
        if self.radioButton_time.isChecked():
            self.timeplot_type = "time"

    def setupPlot(self):
        # Create small plot
        self.timeplot = self.timeplotW.plot()
        self.timeplot.setPen((200,200,100))
        self.timeplotW.setLabel('bottom', 'Time', 's')
        self.timeplotW.setLabel('left', 'Voltage', 'V')
        # Create big plot
        self.rangeplot = self.rangeplotW.plot()
        self.timeplot.setPen((200,200,100))
        self.rangeplotW.setLabel('bottom', 'Distance', 'm')
        self.rangeplotW.setLabel('left', 'Power', 'dB')
        self.last_settings = None
        self.last_update = 0
        self.waterfall_history = None
        self.timeplotx = []

    def update_timeplot(self, y, ffty):
        """Update the small plot"""
        if self.timeplot_type == "none":
            return
        if self.timeplot_type == "freq":
            y = ffty
            self.timeplotW.setLabel('bottom', 'Frequency', 'Hz')
            self.timeplotW.setLabel('left', 'Power', 'dB')
        if self.timeplot_type == "time":
            self.timeplotW.setLabel('bottom', 'Time', 's')
            self.timeplotW.setLabel('left', 'Voltage', 'V')
        if not self.timeplot_valid or len(y) != len(self.timeplotx):
            self.timeplot_valid = True
            if self.timeplot_type == "time":
                new_x = np.linspace(0, 1000*len(y)/self.radar.bb_srate, len(y))
                self.timeplotW.setYRange(-1, 1)
                self.timeplotW.setXRange(0, 1000*len(y)/self.radar.bb_srate)
            elif self.timeplot_type == "freq":
                new_x = np.linspace(0, self.radar.bb_srate/2., len(y))
                self.timeplotW.setYRange(-30, 100)
                self.timeplotW.setXRange(0, new_x[-1])
            else:
                raise Exception("Unhandled plot type {}".format(self.timeplot_type))
            self.timeplotx = new_x
        try:
            self.timeplot.setData(x=self.timeplotx, y=y)
        except Exception:
            self.timeplotW.clear()
            self.timeplot = self.timeplotW.plot(x=self.timeplotx, y=y)

    def update_rangeplot(self, ffty):
        """Update big plot"""
        #d = c f tramp/(2 bw)
        bw = abs(self.radar.fstart-self.radar.fstop)
        tramp = self.radar.sweep_length
        c = 299792458
        if self.rangeplot_type == "range":
            if (bw,tramp, len(ffty)) != self.last_settings or self.rangeplotx == None:
                distance = lambda f: c*f*tramp/(2*bw)
                self.rangeplotx = np.linspace(0, distance(self.radar.bb_srate/2.), len(ffty))
                self.rangeplotW.clear()
                self.rangeplotW.setYRange(-30, 100)
                self.rangeplotW.setXRange(0, self.rangeplotx[-1])
                self.rangeplotW.setLabel('bottom', 'Distance', 'm')
                self.rangeplotW.setLabel('left', 'Power', 'dB')
                self.rangeplot = self.rangeplotW.plot(x=self.rangeplotx, y=ffty)
                self.last_settings = (bw, tramp, len(ffty))
            else:
                self.rangeplot.setData(x=self.rangeplotx, y=ffty)
        elif self.rangeplot_type == "waterfall":
            pass

    @QtCore.Slot(list)
    def plotTimePoints(self, y):
        if abs(time.time() - self.last_update) < 1/30.:
            return
        self.last_update = time.time()
        ffty = 20*np.log10(fft(y))
        self.update_timeplot(y, ffty)
        self.update_rangeplot(ffty)

    def set_rfpower(self):
        power = self.rfpowerBox.isChecked()
        self.radar.set_rfpower(power)

    def set_sweep(self):
        #Fix units
        start = self.startSpinBox.value()*1e9
        stop = self.stopSpinBox.value()*1e9
        length = self.lengthSpinBox.value()*1e-3
        self.radar.set_sweep(start, stop, length)
        #Reset plot
        self.timeplot_valid = False

    def startWorkers(self):
        for worker in self.threads:
            worker.setTerminationEnabled(True)
            worker.start()
            # no wait, no finished. you start the threads and leave.

    def workersFinished(self):
        if all(worker.isFinished() for worker in self.threads):
            # wait until all the threads finished
            pass

    def stopWorkers(self):
        for worker in self.threads:
            worker.stop()

    def closeEvent(self, event):
        self.stopWorkers()
        self.radar.disconnect()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    frame = MainWindow()
    frame.show()
    app.exec_()
