import os
import numpy as np

import rospy
import rosbag

import matplotlib

# Make sure that we are using QT5
matplotlib.use('Qt5Agg')

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

from PyQt5.Qt import Qt
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QScrollArea, QCheckBox, QLabel
from PyQt5.QtCore import QObject, pyqtSignal, QThread


PX_PER_INCH = plt.rcParams['figure.dpi']
FIGURE_WIDTH_INCHES = 12
AX_HEIGHT_INCHES = 2


class PlotTopicTimesWorker(QObject):
    started = pyqtSignal()
    finished = pyqtSignal(plt.Figure)

    def __init__(self, topic_times):
        super().__init__()
        self._topic_times = topic_times

    def run(self):
        self.started.emit()
        fig = plot_topic_times(self._topic_times)
        self.finished.emit(fig)


class PlotTopicTimesWidget(QWidget):
    started_plotting = pyqtSignal()
    finished_plotting = pyqtSignal()

    def __init__(self):
        super().__init__()

        self._enabled = False

        self._checkbox = QCheckBox('Plot message frequency after recording')
        self._display = QLabel()

        self._plot_widget = QWidget()

        self._plot_widget.setLayout(QVBoxLayout())
        self._plot_widget.layout().setContentsMargins(0, 0, 0, 0)
        self._plot_widget.layout().setSpacing(0)

        self._plot_widget.setMinimumHeight(AX_HEIGHT_INCHES * PX_PER_INCH)
        self._plot_widget.setMinimumWidth((1 + FIGURE_WIDTH_INCHES) * PX_PER_INCH)
        self._plot_widget.setMaximumWidth((1 + FIGURE_WIDTH_INCHES) * PX_PER_INCH)

        self._checkbox.stateChanged.connect(self.toggle_plotting)

        self.setLayout(QVBoxLayout())
        self.layout().addWidget(self._checkbox)
        self.layout().addWidget(self._display)
        self.layout().addWidget(self._plot_widget)

    def toggle_plotting(self, state):
        if state == Qt.Checked:
            self._enabled = True
        else:
            self._enabled = False

    def disable_checkbox(self):
        self._checkbox.setEnabled(False)

    def enable_checkbox(self):
        self._checkbox.setEnabled(True)

    def trigger_plotting(self, topic_times):
        if not self._enabled:
            self._display.setText('')
            self._clear_plot()
            return

        self._worker = PlotTopicTimesWorker(topic_times)
        self._thread = QThread()
        self._worker.moveToThread(self._thread)

        self._thread.started.connect(self._worker.run)
        self._worker.finished.connect(self.updated_plot)
        self._worker.finished.connect(self._thread.quit)
        self._worker.finished.connect(self._worker.deleteLater)
        self._thread.finished.connect(self._thread.deleteLater)

        self._worker.started.connect(lambda: self.started_plotting.emit())
        self._worker.finished.connect(lambda: self.finished_plotting.emit())

        self._worker.started.connect(
            lambda: self._display.setText(f'Plotting msg times...')
        )
        self._worker.finished.connect(lambda: self._display.setText(f''))

        self._thread.start()

    def _clear_plot(self):
        while self._plot_widget.layout().count():
            child = self._plot_widget.layout().takeAt(0)
            if child.widget():
                child.widget().deleteLater()

    def updated_plot(self, fig):
        self._clear_plot()

        self._fig = fig
        self.canvas = FigureCanvas(self._fig)
        self.canvas.draw()
        self.scroll = QScrollArea(self)
        self.scroll.setWidget(self.canvas)

        self.nav = NavigationToolbar(self.canvas, self)
        self._plot_widget.layout().addWidget(self.nav)
        self._plot_widget.layout().addWidget(self.scroll)


def plot_topic_times(topic_times):
    fig = plt.figure(figsize=(FIGURE_WIDTH_INCHES, AX_HEIGHT_INCHES * len(topic_times)))

    num_topics = len(topic_times)
    axs = []
    for i in range(num_topics):
        axs.append(fig.add_subplot(num_topics, 1, i + 1))

    topic_times = dict(sorted(topic_times.items(), key=lambda x: x[0]))
    for i, (topic, times) in enumerate(topic_times.items()):

        axs[i].set_title(topic)
        axs[i].set_xlabel('Frequency [Hz]')

        times = np.array(times)

        if len(times) > 0:
            try:
                diffs = np.diff(times)
                freqs = 1 / diffs[diffs != 0.0]
                axs[i].hist(freqs, bins=50)

            except Exception as e:
                axs[i].set_ylim([0, 1])
                rospy.logwarn(f'Could not plot frequency for {topic}')
                print(e)

    plt.subplots_adjust(hspace=1.0)
    fig.tight_layout()
    return fig
