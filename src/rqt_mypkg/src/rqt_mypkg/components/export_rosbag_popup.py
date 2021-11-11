import os
import rospy
import rosbag
import rospkg
from python_qt_binding.QtWidgets import QWidget, QFileDialog, QFormLayout, QHBoxLayout, QPushButton, QLabel, QComboBox, QLineEdit, QCheckBox
from PyQt5.QtGui import QFont
from PyQt5.QtCore import pyqtSignal

class ExportRosbagPopup(QWidget):

    submitted = pyqtSignal(str, str, bool, name='export_rosbag')

    def __init__(self):
        QWidget.__init__(self)
        self.resize(640, 480)
        # self.topic_edit = QLineEdit()
        # self.message_b_edit = QLineEdit()
        self.topic = QLineEdit('annotations')
        self.path = 'No path chosen'
        self.path_display = QLabel(
            text=self.path,
            font=QFont('Sans', 12)
        )
        self.file_button = QPushButton('File')
	self.file_name = QLineEdit('output.bag')
        self.cancel_button = QPushButton('Cancel')
        self.submit_button = QPushButton('Submit')
        self.submit_button.setEnabled(False)
	self.lidar_checkbox = QCheckBox('Export LiDAR data')

        self.setLayout(QFormLayout())
        self.layout().addRow(self.file_button, self.path_display)
        self.layout().addRow('File Name', self.file_name)
        self.layout().addRow('Topic', self.topic)
	self.layout().addRow(self.lidar_checkbox)
        buttons = QWidget()
        buttons.setLayout(QHBoxLayout())
        # buttons.layout().addWidget(self.file_button)
        buttons.layout().addWidget(self.cancel_button)
        buttons.layout().addWidget(self.submit_button)
        self.layout().addRow('', buttons)

        self.file_button.clicked.connect(self.get_rosbag_filename)
        self.submit_button.clicked.connect(self.on_submit)
        self.cancel_button.clicked.connect(self.close)

	# Load in styling for GUI
        style_path = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'dark.qss')
        with open(style_path, 'r') as qss:
            self.style = qss.read()
        self.setStyleSheet(self.style)

    def on_submit(self):
	checked = self.lidar_checkbox.isChecked()
        self.submitted.emit( self.path + '/' + self.file_name.text(), self.topic.text(), checked )
        self.close()

    def get_rosbag_filename(self):
        # Retrieve rosbag path string from file explorer window
        self.path = QFileDialog.getExistingDirectory(self, 'Output Directory', '/home', QFileDialog.ShowDirsOnly)
	self.path_display.setText(self.path)
	self.submit_button.setEnabled(True)
