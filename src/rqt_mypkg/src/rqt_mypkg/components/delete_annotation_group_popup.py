import os
import rospkg
from python_qt_binding.QtWidgets import QWidget, QLabel, QFormLayout, QHBoxLayout, QVBoxLayout, QPushButton, QMessageBox, QComboBox
from PyQt5.QtGui import QFont
from PyQt5.QtCore import pyqtSignal

class DeleteAnnotationGroupPopup(QWidget):

    deleted = pyqtSignal(str, name='delete_annotation_group')

    def __init__(self, annotation_groups):
        QWidget.__init__(self)
        self.resize(500, 150)
        self.setWindowTitle('Delete annotation group')
        self.group_dropdown = QComboBox()
        self.group_dropdown.addItem(None)
        for group in annotation_groups:
            self.group_dropdown.addItem(group.name)
        self.cancel_button = QPushButton('Cancel')
        self.delete_button = QPushButton('Delete')
        self.delete_button.setEnabled(False)

        # Connect buttons to signals and functions
        self.group_dropdown.currentTextChanged.connect(self.enable_delete)
        self.delete_button.clicked.connect(self.on_delete)
        self.cancel_button.clicked.connect(self.close)

        # Create button row widget
        button_widget = QWidget()
        button_widget.setLayout(QHBoxLayout())
        button_widget.layout().addWidget(self.cancel_button)
        button_widget.layout().addWidget(self.delete_button)

        # Add widgets to main layout
        layout = QVBoxLayout()
        layout.setContentsMargins(10, 0, 10, 0)
        layout.addWidget(QLabel(''))
        layout.addWidget(QLabel(text='Annotation group:', font=QFont('Sans', 10)))
        layout.addWidget(self.group_dropdown)
        layout.addWidget(QLabel(''))
        layout.addWidget(QLabel(''))
        layout.addWidget(button_widget)
        self.setLayout(layout)

        # Load in styling for GUI
        style_path = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'dark.qss')
        with open(style_path, 'r') as qss:
            self.style = qss.read()
        self.setStyleSheet(self.style)

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