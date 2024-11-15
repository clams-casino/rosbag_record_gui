from PyQt5.QtWidgets import QPushButton
from PyQt5.QtCore import pyqtSignal


class RecordButtonWidget(QPushButton):
    pushed_start_recording = pyqtSignal()
    pushed_stop_recording = pyqtSignal()

    def __init__(self):
        super().__init__()

        self.setFixedSize(150, 40)

        self.stopped_recording()

        self.clicked.connect(self._on_clicked)

    def _on_clicked(self):
        if not self._recording:
            self.pushed_start_recording.emit()
        else:
            self.pushed_stop_recording.emit()

    def started_recording(self):
        self.setText('Stop Recording')
        self._recording = True

    def stopped_recording(self):
        self.setText('Start Recording')
        self._recording = False

    def disable(self):
        self.setEnabled(False)

    def enable(self):
        self.setEnabled(True)
