import os

from PyQt5.QtWidgets import QMenuBar, QAction, QFileDialog
from PyQt5.QtCore import pyqtSignal


class MenuBar(QMenuBar):
    save_config_signal = pyqtSignal(str)
    load_config_signal = pyqtSignal(str)

    def __init__(self, parent):
        super().__init__(parent)

        self._file_menu = self.addMenu('File')
        self._save_config_action = QAction('Save Config', self)
        self._load_config_action = QAction('Load Config', self)
        self._file_menu.addAction(self._save_config_action)
        self._file_menu.addAction(self._load_config_action)

        self._save_config_action.triggered.connect(self._save_config)
        self._load_config_action.triggered.connect(self._load_config)

    def _save_config(self):
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Config", "", "Config Files (*.yaml)", options=options
        )
        if file_path[-5:] != '.yaml':
            file_path += '.yaml'
        self.save_config_signal.emit(file_path)

    def _load_config(self):
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Load Config", "", "Config Files (*.yaml)", options=options
        )
        if os.path.isfile(file_path):
            self.load_config_signal.emit(file_path)

    def disable_load_config(self):
        self._load_config_action.setEnabled(False)

    def enable_load_config(self):
        self._load_config_action.setEnabled(True)
