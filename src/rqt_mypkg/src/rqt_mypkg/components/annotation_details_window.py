from python_qt_binding.QtWidgets import QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QMessageBox, QLineEdit, QComboBox
from python_qt_binding.QtGui import QDoubleValidator, QPainter
from python_qt_binding.QtCore import Qt
from auxiliary_functions import deleteItemsOfLayout, get_annotation_group_by_name
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot
from visualization_msgs.msg import *
import uuid

class AnnotationDetailsWindow(QWidget):

    confirmed_annotation = pyqtSignal(str, str, str, name='confirmed_annotation')
    cancelled_new_annotation = pyqtSignal(name='cancelled_new_annotation')

    def __init__(self, _annotation_groups):
        QWidget.__init__(self)
        # Set state flags
        self.prompt_new_annotation_flag = False
        self.new_pending_annotation_marker = False
        self.new_pending_annotation_points = False
        # Title
        self.title = QLabel('Annotation Details')
        self.title.setProperty('class', 'widgetTitle')
        self.title.setAlignment(Qt.AlignHCenter)
        # Label
        self.label_input = QLineEdit()

        self.annotation_groups = _annotation_groups
        # Annotation group dropdown
        self.group_dropdown = QComboBox()
        self.group_dropdown.addItem(None)
        for group in self.annotation_groups:
            self.group_dropdown.addItem(group.name)
        self.group_dropdown.currentIndexChanged.connect(self.group_dropdown_change)

        # X plane
        self.x_scale = QLineEdit()
        self.x_scale.setReadOnly(True)
        self.x_position = QLineEdit()
        self.x_position.setReadOnly(True)

        self.x_row = QHBoxLayout()
        self.x_row.addWidget(QLabel('X-scale'))
        self.x_row.addWidget(self.x_scale)
        self.x_row.addWidget(QLabel('X-position'))
        self.x_row.addWidget(self.x_position)
        # Y plane
        self.y_scale = QLineEdit()
        self.y_scale.setReadOnly(True)
        self.y_position = QLineEdit()
        self.y_position.setReadOnly(True)

        self.y_row = QHBoxLayout()
        self.y_row.addWidget(QLabel('Y-scale'))
        self.y_row.addWidget(self.y_scale)
        self.y_row.addWidget(QLabel('Y-position'))
        self.y_row.addWidget(self.y_position)
        # Z plane
        self.z_scale = QLineEdit()
        self.z_scale.setReadOnly(True)
        self.z_position = QLineEdit()
        self.z_position.setReadOnly(True)

        self.z_row = QHBoxLayout()
        self.z_row.addWidget(QLabel('Z-scale'))
        self.z_row.addWidget(self.z_scale)
        self.z_row.addWidget(QLabel('Z-position'))
        self.z_row.addWidget(self.z_position)
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

        self.pending_annotation = {
            'x_scale': None,
            'y_scale': None, 
            'z_scale': None, 
            'x_position': None, 
            'y_position': None, 
            'z_position': None
        }

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
        if not self.prompt_new_annotation_flag:
            # Create the cancel and create buttons and connect to functions
            cancel_button = QPushButton('Cancel')
            self.create_button = QPushButton('Create')
            cancel_button.clicked.connect(self.cancel_new_annotation)
            self.create_button.clicked.connect(self.create_new_annotation)
            # Add buttons to layout
            self.new_annotation_button_layout = QHBoxLayout()
            self.new_annotation_button_layout.addWidget(cancel_button)
            self.new_annotation_button_layout.addWidget(self.create_button)
            self.layout.addLayout(self.new_annotation_button_layout)
        self.prompt_new_annotation_flag = True
        # clear displays
        self.clear_fields()
        # Set X, Y, Z values
        self.x_scale.setText( str(round(self.pending_annotation['x_scale'], 3)) )
        self.y_scale.setText( str(round(self.pending_annotation['y_scale'], 3)) )
        self.z_scale.setText( str(round(self.pending_annotation['z_scale'], 3)) )
        self.x_position.setText( str(round(self.pending_annotation['x_position'], 3)) )
        self.y_position.setText( str(round(self.pending_annotation['y_position'], 3)) )
        self.z_position.setText( str(round(self.pending_annotation['z_position'], 3)) )

    def cancel_new_annotation(self, dont_notify_rviz=False):
        self.clear_fields()
        deleteItemsOfLayout(self.new_annotation_button_layout)
        self.prompt_new_annotation_flag = False
        if not dont_notify_rviz:
            self.cancelled_new_annotation.emit()

    def create_new_annotation(self):
        deleteItemsOfLayout(self.new_annotation_button_layout)
        self.prompt_new_annotation_flag = False
        group_id = get_annotation_group_by_name(self.annotation_groups, self.group_dropdown.currentText()).id
        self.confirmed_annotation.emit( self.label_input.text(), str(uuid.uuid4()), group_id )

    def clear_fields(self):
        self.label_input.clear()
        # Always trigger group_dropdown_change for consistent behavior
        if self.group_dropdown.currentIndex() == 0:
            self.group_dropdown_change()
        else:
            self.group_dropdown.setCurrentIndex(0)
        # Clear X, Y, Z fields
        self.x_scale.setText('')
        self.y_scale.setText('')
        self.z_scale.setText('')
        self.x_position.setText('')
        self.y_position.setText('')
        self.z_position.setText('')

    @pyqtSlot(float, float, float, float, float, float, name='get_pending_annotation_marker')
    def get_pending_annotation_marker(self, x_scale, y_scale, z_scale, x_position, y_position, z_position):
        self.new_pending_annotation_marker = True
        self.pending_annotation['x_scale'] = x_scale
        self.pending_annotation['y_scale'] = y_scale
        self.pending_annotation['z_scale'] = z_scale
        self.pending_annotation['x_position'] = x_position
        self.pending_annotation['y_position'] = y_position
        self.pending_annotation['z_position'] = z_position

        # If the points signal has also been received, enter new annotation mode
        if not self.new_pending_annotation_points:
            self.new_pending_annotation_marker = False
            self.new_pending_annotation_points = False
            self.prompt_new_annotation()
    
    @pyqtSlot(name='get_pending_annotation_points')
    def get_pending_annotation_points(self):
        self.new_pending_annotation_points = True

        # If the points signal has also been received, enter new annotation mode
        if self.new_pending_annotation_marker:
            self.new_pending_annotation_marker = False
            self.new_pending_annotation_points = False
            self.prompt_new_annotation()

    @pyqtSlot(name='rviz_cancelled_new_annotation')
    def rviz_cancelled_new_annotation(self):
        # Reset window without notifying rviz visualization frame
        if self.prompt_new_annotation_flag:
            self.cancel_new_annotation(True)
