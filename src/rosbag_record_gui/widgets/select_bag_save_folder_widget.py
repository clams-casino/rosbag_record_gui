import os
from PyQt5.QtWidgets import QWidget, QPushButton, QLabel, QFileDialog, QHBoxLayout
from PyQt5.QtCore import pyqtSignal


class SelectBagSaveFolderWidget(QWidget):
    selected_signal = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        self._button = QPushButton('Select Bag\nSave Folder')
        self._button.setFixedSize(150, 40)
        self._button.clicked.connect(self._open_folder_dialog)

        self._label = QLabel(f'Saving bags to:\t{os.getcwd()}')

        layout = QHBoxLayout()
        layout.addWidget(self._button)
        layout.addWidget(self._label)
        self.setLayout(layout)

    def _open_folder_dialog(self):
        options = QFileDialog.Options()
        folder_path = QFileDialog.getExistingDirectory(
            self, "Select a folder", "", options=options
        )
        if folder_path and os.path.isdir(folder_path):
            self.selected_signal.emit(folder_path)

    def set_folder_path(self, folder_path):
        self._label.setText(f'Saving bags to:\t{folder_path}')
