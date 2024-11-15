from PyQt5.QtWidgets import QTextEdit


class StatusDisplayWidget(QTextEdit):
    def __init__(self):
        super().__init__()

        self.setMaximumHeight(150)

        self.setReadOnly(True)
        self.setText('Status:\n\n\n')

    def set_status_message(self, message):
        self.setText(f'Status:\n{message}\n\n')
