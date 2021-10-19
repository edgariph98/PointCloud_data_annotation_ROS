from python_qt_binding.QtWidgets import QWidget, QFormLayout, QHBoxLayout, QPushButton, QMessageBox, QComboBox
from PyQt5.QtCore import pyqtSignal

class DeleteAnnotationGroupPopup(QWidget):

    deleted = pyqtSignal(str, name='delete_annotation_group')

    def __init__(self, annotation_groups):
        QWidget.__init__(self)
        self.resize(640, 480)
        self.group_dropdown = QComboBox()
        self.group_dropdown.addItem(None)
        for group in annotation_groups:
            self.group_dropdown.addItem(group.name)
        self.cancel_button = QPushButton('Cancel')
        self.delete_button = QPushButton('Delete')
        self.delete_button.setEnabled(False)


        self.setLayout(QFormLayout())
        self.layout().addRow('Annotation group', self.group_dropdown)
        buttons = QWidget()
        buttons.setLayout(QHBoxLayout())
        buttons.layout().addWidget(self.cancel_button)
        buttons.layout().addWidget(self.delete_button)
        self.layout().addRow('', buttons)

        self.group_dropdown.currentTextChanged.connect(self.enable_delete)
        self.delete_button.clicked.connect(self.on_delete)
        self.cancel_button.clicked.connect(self.close)

    def enable_delete(self):
        if self.group_dropdown.currentText != None:
            self.delete_button.setEnabled(True)
        else:
            self.delete_button.setEnabled(False)

    def on_delete(self):
        # if self.group_dropdown.currentText() != None:
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Warning)
        msg.setText('Are you sure you want to permanitely delete annotation group: {}?'.format(self.group_dropdown.currentText()))
        msg.setWindowTitle('Warning')
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        retval = msg.exec_()
        if retval == QMessageBox.No:
            return
        self.deleted.emit( self.group_dropdown.currentText() )
        self.close()