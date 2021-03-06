from python_qt_binding.QtWidgets import QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QMessageBox, QLineEdit, QComboBox, QTreeView, QAbstractItemView
from python_qt_binding.QtGui import QPixmap, QColor, QIcon, QStandardItemModel, QStandardItem, QBrush
from python_qt_binding.QtCore import Qt
from PyQt5.QtCore import pyqtSlot, pyqtSignal
import os
import rospkg

class AnnotationListWindow(QWidget):

    annotation_details = pyqtSignal(str, str, str, float, float, float, float, float, float, name='annotation_details')

    def __init__(self, _frames, _annotation_groups):
        QWidget.__init__(self)

        self.resource_path = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource')
        icon_path = os.path.join(self.resource_path, 'icons')

        self.frames = _frames
        self.annotation_groups = _annotation_groups
        self.current_annotations = []
        self.current_frame = 0

        # Create annotation group buttons
        button_layout = QHBoxLayout()
        self.create_group_button = QPushButton(QIcon(os.path.join(icon_path, 'plus-thick.svg')), 'Create Group')
        self.create_group_button.setProperty('class', 'dark')
        self.create_group_button.setVisible(False)
        self.delete_group_button = QPushButton(QIcon(os.path.join(icon_path, 'delete.svg')), 'Delete Group')
        self.delete_group_button.setProperty('class', 'dark')
        self.delete_group_button.setVisible(False)
        button_layout.addWidget(self.create_group_button)
        button_layout.addWidget(self.delete_group_button)

        # Create QTreeView and model
        self.treeview = QTreeView(self)
        self.treeview.setStyleSheet("border: none;")
        self.treeview.setHeaderHidden(True)
        self.treeview.setEditTriggers(QAbstractItemView.NoEditTriggers)

        self.model = QStandardItemModel()
        rootNode = self.model.invisibleRootItem()

        self.treeview.setModel(self.model)
        self.treeview.setColumnWidth(0, 150)
        self.treeview.hideColumn(1)

        # Creat spacing label component
        spacing_label = QLabel('')
        spacing_label.setStyleSheet('background-color: rgb(31,30,31);')

        # Add to components to layout
        layout = QVBoxLayout(self)
        layout.setSpacing(0)
        layout.addLayout(button_layout)
        layout.addWidget(spacing_label)
        layout.addWidget(self.treeview) 
        self.setLayout(layout)

        self.treeview.selectionModel().selectionChanged.connect(self.on_selection_change)

    def set_frames(self, _frames):
        self.frames = _frames

    def on_selection_change(self):
        selected_index = self.treeview.currentIndex()
        id_index = selected_index.sibling(selected_index.row(), 1)
        selected_annotation_id = self.model.data( id_index )
        # If id_index is not None an annotation was selected
        if id_index:
            for annotation in self.current_annotations:
                if annotation.id == selected_annotation_id:
                    scale = annotation.get_scale()
                    position = annotation.intMarker.pose.position
                    self.annotation_details.emit(
                        annotation.id, 
                        annotation.label, 
                        annotation.group_id,
                        scale.x,
                        scale.y,
                        scale.z,
                        position.x,
                        position.y,
                        position.z,
                    )
                    break

    @pyqtSlot(int, name='changed_frame')
    def refresh(self, frame_number):
        self.clear_items()
        self.current_frame = frame_number
        self.current_annotations = self.frames[frame_number].annotations
        for group_index, group in enumerate(self.annotation_groups):
            # Create group color icon
            pixmap = QPixmap(100, 100)
            pixmap.fill(QColor(group.color))
            color_icon = QIcon(pixmap)
            # Add annotation group to the tree
            group_row = QStandardItem(color_icon, group.name)
            self.model.appendRow([group_row, None])
            # add annotations for this frame under this group
            for annotation in self.current_annotations:
                if annotation.group_id == group.id:
                    display_name = annotation.label if annotation.label else annotation.id
                    group_row.appendRow([QStandardItem(display_name), QStandardItem(annotation.id)])
        self.treeview.hideColumn(1)

    @pyqtSlot(str, name='delete_annotation')
    def delete_annotation(self, id):
        self.refresh(self.current_frame)

    def clear_items(self):
        if self.model.hasChildren():
            self.model.removeRows(0, self.model.rowCount())

    def display_buttons(self):
        self.create_group_button.setVisible(True)
        self.delete_group_button.setVisible(True)
