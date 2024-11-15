from PyQt5.QtWidgets import QWidget, QHBoxLayout, QPushButton
from PyQt5.QtCore import pyqtSignal


class RecordButtonsWidget(QWidget):
    pushed_start_recording = pyqtSignal()
    pushed_stop_recording = pyqtSignal()

    pushed_delete_bag = pyqtSignal()

    def __init__(self):
        super().__init__()

        self._record_button = QPushButton('Start Recording')
        self._record_button.setFixedSize(150, 40)

        self._delete_button = QPushButton('Delete Last Bag')
        self._delete_button.setFixedSize(150, 40)

        layout = QHBoxLayout()
        layout.addWidget(self._record_button)
        layout.addWidget(self._delete_button)
        self.setLayout(layout)

        self.stopped_recording()

        self._record_button.clicked.connect(self._record_button_clicked)
        self._delete_button.clicked.connect(self._delete_button_clicked)

    def _record_button_clicked(self):
        if not self._recording:
            self.pushed_start_recording.emit()
        else:
            self.pushed_stop_recording.emit()

    def _delete_button_clicked(self):
        self.pushed_delete_bag.emit()

    def started_recording(self):
        self._record_button.setText('Stop Recording')
        self._delete_button.setEnabled(False)
        self._recording = True

    def stopped_recording(self):
        self._record_button.setText('Start Recording')
        self._delete_button.setEnabled(True)
        self._recording = False

    def disable(self):
        self._record_button.setEnabled(False)
        self._delete_button.setEnabled(False)

    def enable(self):
        self._record_button.setEnabled(True)
        self._delete_button.setEnabled(True)
