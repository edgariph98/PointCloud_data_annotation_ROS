from python_qt_binding.QtWidgets import QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QMessageBox, QLineEdit, QComboBox
from python_qt_binding.QtGui import QDoubleValidator, QPainter
from python_qt_binding.QtCore import Qt
from .auxiliary_functions import deleteItemsOfLayout, get_annotation_group_by_name, get_annotation_group_by_id
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import pyqtSlot
from visualization_msgs.msg import *
import uuid

class AnnotationDetailsWindow(QWidget):

    confirmed_annotation = pyqtSignal(str, str, str, name='confirmed_annotation')
    cancelled_new_annotation = pyqtSignal(name='cancelled_new_annotation')
    confirmed_delete_annotation = pyqtSignal(str, name='confirmed_delete_annotation')

    def __init__(self, _annotation_groups):
        QWidget.__init__(self)
        self.annotation_groups = _annotation_groups
        self.current_annotation_id = ''
        # Set state flags
        self.prompt_new_annotation_flag = False
        self.showing_details_flag = False
        # Title
        self.title = QLabel('Annotation Details')
        self.title.setProperty('class', 'widgetTitle')
        self.title.setAlignment(Qt.AlignHCenter)
        # Label
        self.label_input = QLineEdit()

        # Annotation group dropdown
        self.group_dropdown = QComboBox()
        self.group_dropdown.addItem(None)
        for group in self.annotation_groups:
            self.group_dropdown.addItem(group.name)
        self.group_dropdown.currentIndexChanged.connect(self.group_dropdown_change)
        self.group_display = QLineEdit()
        self.group_display.setReadOnly(True)
        self.group_display.setVisible(False)

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
        self.layout.addWidget(self.group_display)
        self.layout.addLayout(self.x_row)
        self.layout.addLayout(self.y_row)
        self.layout.addLayout(self.z_row)
        # self.layout.addWidget(self.delete_button_row)
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
        if self.showing_details_flag:
            self.showing_details_flag = False
            deleteItemsOfLayout(self.delete_button_layout)
        # Set title
        self.title.setText('Create Annotation')
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
        new_annotation_id = str(uuid.uuid4())
        self.confirmed_annotation.emit( self.label_input.text(), new_annotation_id, group_id )

        self.get_annotation_details(
            new_annotation_id,
            self.label_input.text(),
            group_id,
            self.pending_annotation['x_scale'],
            self.pending_annotation['y_scale'],
            self.pending_annotation['z_scale'],
            self.pending_annotation['x_position'],
            self.pending_annotation['y_position'],
            self.pending_annotation['z_position']
        )

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
        # Reactivate inputs
        self.group_dropdown.setVisible(True)
        self.group_display.setVisible(False)
        self.label_input.setReadOnly(False)
        # When changing frames and window is in details mode remove delete button
        if self.showing_details_flag:
            self.showing_details_flag = False
            deleteItemsOfLayout(self.delete_button_layout)

    @pyqtSlot(float, float, float, float, float, float, name='get_pending_annotation_marker')
    def get_pending_annotation_marker(self, x_scale, y_scale, z_scale, x_position, y_position, z_position):
        self.pending_annotation['x_scale'] = x_scale
        self.pending_annotation['y_scale'] = y_scale
        self.pending_annotation['z_scale'] = z_scale
        self.pending_annotation['x_position'] = x_position
        self.pending_annotation['y_position'] = y_position
        self.pending_annotation['z_position'] = z_position
        self.prompt_new_annotation()

    @pyqtSlot(name='rviz_cancelled_new_annotation')
    def rviz_cancelled_new_annotation(self):
        # Reset window without notifying rviz visualization frame
        if self.prompt_new_annotation_flag:
            self.cancel_new_annotation(True)
            
    @pyqtSlot(str, str, str, float, float, float, float, float, float, name='annotation_details')
    def get_annotation_details(self, id, label, group_id, x_scale, y_scale, z_scale, x_position, y_position, z_position):
        if not self.prompt_new_annotation_flag:
            self.current_annotation_id = id
            if not self.showing_details_flag:
                self.showing_details_flag = True
                # Create delete button and add to layout
                self.delete_button_layout = QHBoxLayout()
                delete_button = QPushButton('Delete')
                delete_button.clicked.connect(self.delete_annotation)
                self.delete_button_layout.addWidget(QLabel(''))
                self.delete_button_layout.addWidget(delete_button)
                self.layout.addLayout(self.delete_button_layout)

            # Change the title
            self.title.setText('Annotation Details')
            # Hide the group dropdown, set the group display and unhide it
            group_name = get_annotation_group_by_id(self.annotation_groups, group_id).name
            self.group_display.setText(group_name)
            self.group_display.setVisible(True)
            self.group_dropdown.setVisible(False)

            # Set the values for the rest of the fields
            self.label_input.setText(label)
            self.label_input.setReadOnly(True)
            self.x_scale.setText( str(round(x_scale, 3)) )
            self.y_scale.setText( str(round(y_scale, 3)) )
            self.z_scale.setText( str(round(z_scale, 3)) )
            self.x_position.setText( str(round(x_position, 3)) )
            self.y_position.setText( str(round(y_position, 3)) )
            self.z_position.setText( str(round(z_position, 3)) )

    def delete_annotation(self, id=None):
        # If delete button was pressed (not id)
        # or the SELECTED annotation is deleted using RVIZ (id==self.current_annotation_id), then clear the window
        if not id or id == self.current_annotation_id:
            self.showing_details_flag = False
            deleteItemsOfLayout(self.delete_button_layout)
            self.clear_fields()
            # If ID is not None, signal was sent from Annotator, so no need to emit signal
            if not id:
                # Send annotation ID to the annotator so it can be deleted
                # Signal is also sent to list window so it will update
                self.confirmed_delete_annotation.emit(self.current_annotation_id)
        