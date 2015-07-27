import numpy as np
import sys
from PySide.QtGui import QApplication, QMainWindow, QVBoxLayout
from PySide import QtCore
import dummy_radar
import time

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
        self.job_input = radar
        self.p = None

    def run(self):
        queue = Queue()
        if use_multiprocessing:
            self.p = Process(target=radar_streamer, args=(queue, self.job_input))
            #Closes the process when application closes
            self.p.daemon = True
            self.p.start()
            while True:
                msg = queue.get()
                self.message.emit(msg)
                #Time for updating the GUI
                time.sleep(1/60.)
        else:
            for sample in radar.stream():
                self.message.emit(sample)

    def stop(self):
        if self.p:
            self.p.terminate()
        self.terminate()

def fft(data, window=None):
    if window:
        w = window(len(data))
        data = [data[i]*w[i] for i in xrange(len(w))]
    y = map(abs,np.fft.rfft(data))
    return y

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, radar, parent=None):
        self.threads = []
        self.radar = radar
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.startSpinBox.valueChanged.connect(self.set_sweep)
        self.stopSpinBox.valueChanged.connect(self.set_sweep)
        self.lengthSpinBox.valueChanged.connect(self.set_sweep)

        #Make sure radar and GUI have same settings
        self.set_sweep()

        #Initialize time plot
        self.setupPlot()

        self.timeplot_type = None
        self.radioButton_range.toggled.connect(self.set_timeplot)
        self.radioButton_time.toggled.connect(self.set_timeplot)
        self.radioButton_none.toggled.connect(self.set_timeplot)

        self.radioTrigSingle.toggled.connect(self.set_trig)
        self.radioTrigCont.toggled.connect(self.set_trig)
        self.pushButtonTrigger.clicked.connect(self.trigger_once)

        self.rfpowerBox.toggled.connect(self.set_rfpower)

        self.set_timeplot()

        self.addWorker(Runner(self.radar))
        self.startWorkers()

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

    def set_timeplot(self):
        """Change time plot configuration"""
        self.lines = None
        if self.radioButton_none.isChecked():
            self.timeplot_type = "none"
        if self.radioButton_range.isChecked():
            self.timeplot_type = "range"
        if self.radioButton_time.isChecked():
            self.timeplot_type = "time"

    def setupPlot(self):
        # Create small plot
        self.timeplot = self.timeplotW.plot()
        self.timeplot.setPen((200,200,100))

    @QtCore.Slot(list)
    def plotTimePoints(self, y):
        if self.timeplot_type == "none":
            return
        if self.timeplot_type == "range":
            y = 20*np.log10(fft(y))
        if self.lines == None or len(y) != len(self.timeplotx):
            self.lines = True
            print "Set axes"
            if self.timeplot_type == "time":
                new_x = np.linspace(0, 1000*len(y)/self.radar.bb_srate, len(y))
                self.timeplotW.setYRange(-1, 1)
                self.timeplotW.setXRange(0, 1000*len(y)/self.radar.bb_srate)
            elif self.timeplot_type == "range":
                new_x = np.linspace(0, self.radar.bb_srate/2., len(y))
                self.timeplotW.setYRange(-30, 100)
                self.timeplotW.setXRange(0, new_x[-1])
            self.timeplotx = new_x
        try:
            self.timeplot.setData(x=self.timeplotx, y=y)
        except Exception:
            self.timeplot = self.timeplotW.plot(x=self.timeplotx, y=y)

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
        self.lines = None

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

if __name__ == "__main__":
    radar = dummy_radar.Dummy_radar()

    app = QApplication(sys.argv)
    frame = MainWindow(radar)
    frame.show()
    app.exec_()
