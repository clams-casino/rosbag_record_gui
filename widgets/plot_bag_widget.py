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


class PlotBagWorker(QObject):
    started = pyqtSignal()
    finished = pyqtSignal(plt.Figure)

    def __init__(self, bag_path):
        super().__init__()
        self._bag_path = bag_path

    def run(self):
        # print("IN worker run")
        self.started.emit()
        fig = plot_bag_freq(self._bag_path)
        self.finished.emit(fig)


class PlotBagtWidget(QWidget):
    started_plotting = pyqtSignal()
    finished_plotting = pyqtSignal()

    def __init__(self):
        super().__init__()

        self._enabled = False

        self._checkbox = QCheckBox('plot recorded bag')
        self._display = QLabel()

        self._plot_widget = QWidget()

        self._plot_widget.setLayout(QVBoxLayout())
        self._plot_widget.layout().setContentsMargins(0, 0, 0, 0)
        self._plot_widget.layout().setSpacing(0)

        self._plot_widget.setMinimumHeight(AX_HEIGHT_INCHES * PX_PER_INCH)
        self._plot_widget.setMinimumWidth((1 + FIGURE_WIDTH_INCHES) * PX_PER_INCH)
        self._plot_widget.setMaximumWidth((1 + FIGURE_WIDTH_INCHES) * PX_PER_INCH)

        # self._fig = plt.figure(figsize=(FIGURE_WIDTH_INCHES, AX_HEIGHT_INCHES))
        # self.updated_plot(self._fig)

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

    def trigger_plotting(self, bag_path):
        # print("IN trigger_plotting")

        if not self._enabled:
            self._display.setText('')
            self._clear_plot()
            return


        self._worker = PlotBagWorker(bag_path)
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
            lambda: self._display.setText(
                f'Plotting bag at {bag_path}\nThis may take a while for large bags...'
            )
        )
        self._worker.finished.connect(
            lambda: self._display.setText(f'Finished plotting bag at {bag_path}')
        )

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


def plot_bag_freq(bag_path):
    rospy.loginfo('Started reading bag for plotting')

    if not os.path.exists(bag_path):
        if os.path.exists(bag_path + '.active'):
            rospy.logwarn(f'Cannot plot bag file which is still being written to, waiting for it to finish')
            tries = 0
            while os.path.exists(bag_path + '.active'):
                if tries > 10:
                    raise FileNotFoundError(f'Cannot plot {bag_path} which is still being written to')
                rospy.sleep(1)
                tries += 1
        else:
            raise FileNotFoundError(f'Bag file {bag_path} does not exist')
            
    bag = rosbag.Bag(
        bag_path,
        skip_index=False,  # doesn't really make a difference since we'll have to read index to get the time stamps anyways
    )

    # not skipping the index allows us to get the time start and end times easily
    start_time = bag.get_start_time()
    end_time = bag.get_end_time()

    topics_stamps = {}
    for topic, _, t in bag.read_messages(
        raw=True
    ):  # reading non-deserialized messages is faster
        if topic not in topics_stamps:
            topics_stamps[topic] = [t.to_sec() - start_time]
        else:
            topics_stamps[topic].append(t.to_sec() - start_time)

    bag.close()

    rospy.loginfo('Finished reading bag for plotting')

    rospy.loginfo('Started plotting')

    fig = plt.figure(
        figsize=(FIGURE_WIDTH_INCHES, AX_HEIGHT_INCHES * len(topics_stamps))
    )

    num_topics = len(topics_stamps)
    axs = []
    for i in range(num_topics):
        axs.append(fig.add_subplot(num_topics, 1, i + 1))

    topics_stamps = dict(sorted(topics_stamps.items(), key=lambda x: x[0]))
    for i, (topic, stamps) in enumerate(topics_stamps.items()):

        axs[i].set_title(topic)
        axs[i].set_xlim(0, end_time - start_time)
        axs[i].set_ylabel('freq [Hz]')

        stamps = np.array(stamps)
        if len(stamps) > 0:
            try:
                freqs = 1 / np.diff(stamps)
                inf_idx = np.isinf(freqs)
                axs[i].plot(stamps[-len(freqs):][~inf_idx], freqs[~inf_idx], c='b')
                axs[i].set_title(topic)
                axs[i].set_xlim(0, end_time - start_time)
                axs[i].set_ylim([0, 2 * np.mean(freqs[~inf_idx])])

            except Exception as e:
                axs[i].set_ylim([0, 1])
                rospy.logwarn(f'Could not plot frequency for {topic} in {bag_path}')
                print(e)

        axs[i].stem(
            stamps,
            np.ones_like(stamps) * axs[i].get_ylim()[1] * 0.1,
            linefmt='r-',
            markerfmt='rx',
            use_line_collection=True,
        )

        if i == len(topics_stamps) - 1:
            axs[i].set_xlabel('time [s]')

    plt.subplots_adjust(hspace=1.0)
    fig.tight_layout()
    return fig
