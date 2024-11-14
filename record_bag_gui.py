import sys
import signal

import rospy

from backend import GUIBackend
from widgets import (
    TopicSelectionTreeWidget,
    RecordButtonsWidget,
    StatusDisplayWidget,
    SelectBagSaveFolderWidget,
    PlotTopicTimesWidget,
    MenuBar,
)

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout


def sigint_handler(sig, frame):
    print(
        '\nPlease close this application from the GUI instead of using Ctrl+C'
    )

signal.signal(signal.SIGINT, sigint_handler)


class RecordBagGUI(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle('Record Rosbag GUI')
        self.setGeometry(100, 100, 1000, 1000)

        # create backend
        self._backend = GUIBackend()

        # create widgets
        self._widget = QWidget()
        self.setCentralWidget(self._widget)

        self._topic_selection_tree = TopicSelectionTreeWidget()
        self._status_display = StatusDisplayWidget()
        self._record_buttons = RecordButtonsWidget()
        self._bag_savedir_selection = SelectBagSaveFolderWidget()
        self._plot_bag = PlotTopicTimesWidget()

        buttons_widget = QWidget()
        buttons_layout = QHBoxLayout()
        buttons_layout.addWidget(self._record_buttons)
        buttons_layout.addWidget(self._bag_savedir_selection)
        buttons_widget.setLayout(buttons_layout)

        layout = QVBoxLayout()
        layout.addWidget(self._topic_selection_tree)
        layout.addWidget(self._status_display)
        layout.addWidget(buttons_widget)
        layout.addWidget(self._plot_bag)
        self._widget.setLayout(layout)

        # menu bar
        self._menu_bar = MenuBar(parent=self)
        self.setMenuBar(self._menu_bar)

        # connect signals and slots
        self._backend.add_topics_signal.connect(self._topic_selection_tree.add_topics)
        self._backend.remove_topics_signal.connect(self._topic_selection_tree.remove_topics)
        self._backend.check_topics_signal.connect(self._topic_selection_tree.check_topics)

        self._topic_selection_tree.checked_topic.connect(self._backend.add_topic)
        self._topic_selection_tree.unchecked_topic.connect(self._backend.remove_topic)

        self._record_buttons.pushed_start_recording.connect(self._backend.start_recording)
        self._record_buttons.pushed_stop_recording.connect(self._backend.stop_recording)
        self._record_buttons.pushed_delete_bag.connect(self._backend.delete_bag)

        self._backend.started_recording_signal.connect(self._record_buttons.started_recording)
        self._backend.stopped_recording_signal.connect(self._record_buttons.stopped_recording)
        self._backend.started_recording_signal.connect(self._topic_selection_tree.freeze_selections)
        self._backend.stopped_recording_signal.connect(self._topic_selection_tree.unfreeze_selections)

        self._backend.status_message_signal.connect(self._status_display.set_status_message)

        self._backend.topics_stats_signal.connect(self._topic_selection_tree.set_topics_stats)

        self._bag_savedir_selection.selected_signal.connect(self._backend.set_bag_save_folder)
        self._backend.set_bag_savedir_signal.connect(self._bag_savedir_selection.set_folder_path)

        self._menu_bar.save_config_signal.connect(self._backend.save_config)
        self._menu_bar.load_config_signal.connect(self._backend.load_config)
        self._backend.started_recording_signal.connect(self._menu_bar.disable_load_config)
        self._backend.stopped_recording_signal.connect(self._menu_bar.enable_load_config)

        self._backend.stopped_recording_signal.connect(self._plot_bag.trigger_plotting)
        self._backend.started_recording_signal.connect(self._plot_bag.disable_checkbox)
        self._backend.stopped_recording_signal.connect(self._plot_bag.enable_checkbox)

        self._plot_bag.started_plotting.connect(self._record_buttons.disable)
        self._plot_bag.finished_plotting.connect(self._record_buttons.enable)

    def closeEvent(self, event):
        self._backend.close()
        rospy.signal_shutdown('Closing GUI')
        event.accept()


if __name__ == '__main__':
    rospy.init_node(
        'record_bag_gui',
        anonymous=True,
        disable_signals=True,  # since we're running in a PyQt app
    )
    app = QApplication(sys.argv)
    window = RecordBagGUI()
    window.show()
    sys.exit(app.exec_())
