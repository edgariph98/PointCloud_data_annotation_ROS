import os
import rospkg
from python_qt_binding.QtWidgets import QColorDialog, QLabel, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLineEdit, QMessageBox
from PyQt5.QtGui import QColor, QFont, QPixmap
from PyQt5.QtCore import pyqtSignal

class CreateAnnotationGroupPopup(QWidget):

    created = pyqtSignal(str, QColor, name='create_annotation_group')

    def __init__(self, annotation_groups):
        QWidget.__init__(self)
        self.annotation_groups = annotation_groups
        self.color = None
        self.resize(500, 250)
        self.setWindowTitle('Create annotation group')
        self.group_name_edit = QLineEdit()
        self.color_selector = QPushButton('Browse colors')
        self.color_display = QLabel('')
        self.cancel_button = QPushButton('Cancel')
        self.submit_button = QPushButton('Submit')
        self.submit_button.setEnabled(False)

        # Connect buttons to signals and functions
        self.group_name_edit.textChanged.connect(self.enable_submit)
        self.color_selector.clicked.connect(self.get_color)
        self.submit_button.clicked.connect(self.on_submit)
        self.cancel_button.clicked.connect(self.close)

        # Create color selection row widget
        color_widget = QWidget()
        color_widget.setLayout(QHBoxLayout())
        color_widget.layout().addWidget(self.color_selector)
        color_widget.layout().addWidget(self.color_display)

        # Create button row widget
        button_widget = QWidget()
        button_widget.setLayout(QHBoxLayout())
        button_widget.layout().addWidget(self.cancel_button)
        button_widget.layout().addWidget(self.submit_button)

        # Add widgets to main layout
        layout = QVBoxLayout()
        layout.setContentsMargins(10, 0, 10, 0)
        layout.addWidget(QLabel(''))
        layout.addWidget(QLabel(text='Annotation group name:', font=QFont('Sans', 10)))
        layout.addWidget(self.group_name_edit)
        layout.addWidget(QLabel(''))
        layout.addWidget(QLabel(''))
        layout.addWidget(QLabel(text='Annotation color:', font=QFont('Sans', 10)))
        # layout.addWidget(self.color_selector)
        layout.addWidget(color_widget)
        layout.addWidget(QLabel(''))
        layout.addWidget(QLabel(''))
        layout.addWidget(button_widget)
        self.setLayout(layout)


        # Load in styling for GUI
        style_path = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'dark.qss')
        with open(style_path, 'r') as qss:
            self.style = qss.read()
        self.setStyleSheet(self.style)

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
        # Get and display the selected color
        self.color = QColorDialog.getColor()
        pixmap = QPixmap(220, 25)
        pixmap.fill(self.color)
        self.color_display.setPixmap(pixmap)

        self.enable_submit()