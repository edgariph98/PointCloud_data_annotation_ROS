from python_qt_binding.QtWidgets import QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QMessageBox, QLineEdit, QComboBox
from python_qt_binding.QtGui import QDoubleValidator, QPainter
from python_qt_binding.QtCore import Qt
from auxiliary_functions import deleteItemsOfLayout
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot

class AnnotationDetailsWindow(QWidget):

    confirmed_annotation = pyqtSignal(str, str, name='confirmed_annotation')

    def __init__(self, annotation_groups):
        QWidget.__init__(self)
        # Set state flags
        self.prompt_new_annotation_flag = False
        # Title
        self.title = QLabel('Annotation Details')
        self.title.setProperty('class', 'widgetTitle')
        self.title.setAlignment(Qt.AlignHCenter)
        # Label
        self.label_input = QLineEdit()
        # self.label_input.setProperty('Test', True)
        # self.label_input.setProperty('class', 'Test')
        # self.setStyle(self.style())

        # Annotation group dropdown
        self.group_dropdown = QComboBox()
        self.group_dropdown.addItem(None)
        for group in annotation_groups:
            self.group_dropdown.addItem(group.name)
        self.group_dropdown.currentIndexChanged.connect(self.group_dropdown_change)
        self.onlyDouble = QDoubleValidator()
        # X input
        self.x_min = QLineEdit()
        self.x_min.setValidator(self.onlyDouble)
        self.x_max = QLineEdit()
        self.x_max.setValidator(self.onlyDouble)

        self.x_row = QHBoxLayout()
        self.x_row.addWidget(QLabel('X-min'))
        self.x_row.addWidget(self.x_min)
        self.x_row.addWidget(QLabel('X-max'))
        self.x_row.addWidget(self.x_max)
        # Y input
        self.y_min = QLineEdit()
        self.y_min.setValidator(self.onlyDouble)
        self.y_max = QLineEdit()
        self.y_max.setValidator(self.onlyDouble)

        self.y_row = QHBoxLayout()
        self.y_row.addWidget(QLabel('Y-min'))
        self.y_row.addWidget(self.y_min)
        self.y_row.addWidget(QLabel('Y-max'))
        self.y_row.addWidget(self.y_max)
        # Z input
        self.z_min = QLineEdit()
        self.z_min.setValidator(self.onlyDouble)
        self.z_max = QLineEdit()
        self.z_max.setValidator(self.onlyDouble)

        self.z_row = QHBoxLayout()
        self.z_row.addWidget(QLabel('Z-min'))
        self.z_row.addWidget(self.z_min)
        self.z_row.addWidget(QLabel('Z-max'))
        self.z_row.addWidget(self.z_max)
        # Layout
        self.layout = QVBoxLayout()
        # self.layout.addStretch(0)
        # self.layout.setSpacing(0)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.addWidget(self.title)
        self.layout.addWidget(QLabel('Label'))
        self.layout.addWidget(self.label_input)
        self.layout.addWidget(QLabel('Annotation group'))
        self.layout.addWidget(self.group_dropdown)
        self.layout.addLayout(self.x_row)
        self.layout.addLayout(self.y_row)
        self.layout.addLayout(self.z_row)
        self.layout.addStretch(0)
        self.layout.setSpacing(10)
        self.setLayout(self.layout)

    def group_dropdown_change(self):
        if self.prompt_new_annotation_flag and not self.group_dropdown.currentText():
            self.create_button.setEnabled(False)
        elif self.prompt_new_annotation_flag:
            self.create_button.setEnabled(True)

    def add_annotation_group(self, new_annotation_group_name):
        self.group_dropdown.addItem(new_annotation_group_name)

    def delete_annotation_group(self, annotation_group_name):
        index = self.group_dropdown.findText(annotation_group_name)
        self.group_dropdown.removeItem(index)

    def prompt_new_annotation(self):
        self.prompt_new_annotation_flag = True
        # Create the cancel and create buttons and connect to functions
        cancel_button = QPushButton('Cancel')
        self.create_button = QPushButton('Create')
        cancel_button.clicked.connect(self.cancel_new_annotation)
        self.create_button.clicked.connect(self.create_new_annotation)
        # clear displays
        self.clear_fields()
        # Add buttons to layout
        self.new_annotation_button_layout = QHBoxLayout()
        self.new_annotation_button_layout.addWidget(cancel_button)
        self.new_annotation_button_layout.addWidget(self.create_button)
        self.layout.addLayout(self.new_annotation_button_layout)

    def cancel_new_annotation(self):
        deleteItemsOfLayout(self.new_annotation_button_layout)
        self.prompt_new_annotation_flag = False
        return

    def create_new_annotation(self):
        deleteItemsOfLayout(self.new_annotation_button_layout)
        self.prompt_new_annotation_flag = False
        self.confirmed_annotation.emit( self.label_input.text(), self.group_dropdown.currentText() )
        return

    def clear_fields(self):
        self.label_input.clear()
        # Always trigger group_dropdown_change for consistent behavior
        if self.group_dropdown.currentIndex() == 0:
            self.group_dropdown_change()
        else:
            self.group_dropdown.setCurrentIndex(0)

    @pyqtSlot(float, float, float, float, float, float, name='get_pending_annotation_marker')
    def get_pending_annotation_maker(self, scale_x, scale_y, scale_z, position_x, position_y, position_z):
        print(scale_x)
        print(position_x)