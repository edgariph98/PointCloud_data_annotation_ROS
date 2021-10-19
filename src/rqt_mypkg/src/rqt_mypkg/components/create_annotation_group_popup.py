from python_qt_binding.QtWidgets import QColorDialog, QWidget, QFormLayout, QHBoxLayout, QPushButton, QLineEdit, QMessageBox
from PyQt5.QtGui import QColor
from PyQt5.QtCore import pyqtSignal

class CreateAnnotationGroupPopup(QWidget):

    created = pyqtSignal(str, QColor, name='create_annotation_group')

    def __init__(self, annotation_groups):
        QWidget.__init__(self)
        self.annotation_groups = annotation_groups
        self.color = None
        self.resize(640, 480)
        self.group_name_edit = QLineEdit()
        self.color_selector = QPushButton('Color')
        self.cancel_button = QPushButton('Cancel')
        self.submit_button = QPushButton('Submit')
        self.submit_button.setEnabled(False)


        self.setLayout(QFormLayout())
        self.layout().addRow('Annotation group name', self.group_name_edit)
        self.layout().addRow('Annotation color', self.color_selector)
        buttons = QWidget()
        buttons.setLayout(QHBoxLayout())
        buttons.layout().addWidget(self.cancel_button)
        buttons.layout().addWidget(self.submit_button)
        self.layout().addRow('', buttons)

        self.group_name_edit.textChanged.connect(self.enable_submit)
        self.color_selector.clicked.connect(self.get_color)
        self.submit_button.clicked.connect(self.on_submit)
        self.cancel_button.clicked.connect(self.close)

    def enable_submit(self):
        if len(self.group_name_edit.text()) > 0 and self.color != None :
            self.submit_button.setEnabled(True)
        else:
            self.submit_button.setEnabled(False)

    def on_submit(self):
        for group in self.annotation_groups:
            if self.group_name_edit.text() == group.name:
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Critical)
                msg.setText('An annotation group already exists with this name.')
                msg.setInformativeText('Please change the annotation group name.')
                msg.setWindowTitle('Duplicate Name')
                msg.setStandardButtons(QMessageBox.Ok)
                retval = msg.exec_()
                return
            if self.color == group.color:
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Critical)
                msg.setText('An annotation group already exists with this color.')
                msg.setInformativeText('Please change the annotation group color.')
                msg.setWindowTitle('Duplicate Color')
                msg.setStandardButtons(QMessageBox.Ok)
                retval = msg.exec_()
                return
        self.created.emit( self.group_name_edit.text(), self.color )
        self.close()

    def get_color(self):
        self.color = QColorDialog.getColor()
        self.enable_submit()