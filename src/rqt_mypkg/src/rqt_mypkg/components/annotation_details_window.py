from python_qt_binding.QtWidgets import QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QMessageBox, QLineEdit, QComboBox
from python_qt_binding.QtGui import QDoubleValidator, QPainter
from python_qt_binding.QtCore import Qt

class AnnotationDetailsWindow(QWidget):

    def __init__(self, annotation_groups):
        QWidget.__init__(self)
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


    def add_annotation_group(self, new_annotation_group):
        self.group_dropdown.addItem(new_annotation_group.name)