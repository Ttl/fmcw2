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

import pyqtgraph as pg

matplotlib.use('Qt4Agg')
matplotlib.rcParams['backend.qt4']='PySide'

pg.setConfigOptions(antialias=True)

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
        self.p = Process(target=radar_streamer, args=(queue, self.job_input))
        #Closes the process when application closes
        self.p.daemon = True
        self.p.start()
        while True:
            msg = queue.get()
            self.message.emit(msg)
            #Time for updating the GUI
            time.sleep(1/60.)

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

        #Initialize time plot
        self.setupPlot()

        self.timeplot_type = None
        self.radioButton_range.toggled.connect(self.set_timeplot)
        self.radioButton_time.toggled.connect(self.set_timeplot)
        self.radioButton_none.toggled.connect(self.set_timeplot)
        self.set_timeplot()

        self.addWorker(Runner(self.radar))
        self.startWorkers()

    def addWorker(self, worker):
        worker.message.connect(self.plotTimePoints, QtCore.Qt.QueuedConnection)
        # connect the finished signal to method so that we are notified
        worker.finished.connect(self.workersFinished)
        self.threads.append(worker)

    def set_timeplot(self):
        """Change time plot configuration"""
        self.lines = None
        if self.radioButton_none.isChecked():
            self.timeplot_type = "none"
            self.timeplot.axes.clear()
            self.timeplot.draw()
        if self.radioButton_range.isChecked():
            self.timeplot_type = "range"
        if self.radioButton_time.isChecked():
            self.timeplot_type = "time"


    def setupPlot(self):
        # create a matplotlib widget
        self.timeplot = MatplotlibWidget()
        # create a layout inside the blank widget and add the matplotlib widget
        layout = QVBoxLayout(self.timePlot_area)
        layout.addWidget(self.timeplot, 1)

        self.timeplot.axes.plot([0],[0],'b-')
        self.timeplot.draw()
        self.timeplot.axes.set_axis_bgcolor('w')
        self.timeplot.figure.patch.set_facecolor('white')
        self.timeplot.figure.patch.set_alpha(1.0)

    @QtCore.Slot(list)
    def plotTimePoints(self, y):
        if self.timeplot_type == "none":
            return
        if self.timeplot_type == "range":
            y = 20*np.log10(fft(y))
        if self.lines == None:
            self.timeplot.axes.clear()
            if self.timeplot_type == "time":
                new_x = np.linspace(0, self.radar.sweep_length, len(y))
                self.timeplot.axes.set_ylim(-1,1)
                self.timeplot.axes.set_xlim(0,self.radar.sweep_length)
            elif self.timeplot_type == "range":
                new_x = np.linspace(0, len(y)/self.radar.sweep_length, len(y))
                self.timeplot.axes.set_ylim(-30,100)
                self.timeplot.axes.set_xlim(0,new_x[-1]/2.)
            self.lines, = self.timeplot.axes.plot(new_x,y,'b-')
            self.timeplot.draw()
        else:
            self.lines.set_ydata(y)
            self.timeplot.axes.draw_artist(self.timeplot.axes.patch)
            self.timeplot.axes.draw_artist(self.lines)
            self.timeplot.figure.canvas.update()
            self.timeplot.figure.canvas.flush_events()
        self.timeplot.draw()


    def set_sweep(self):
        start = self.startSpinBox.value()
        stop = self.stopSpinBox.value()
        length = self.lengthSpinBox.value()
        self.radar.set_sweep(start, stop, length)

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
