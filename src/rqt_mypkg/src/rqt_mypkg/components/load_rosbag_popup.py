import os
import rospy
import rosbag
import rospkg
from python_qt_binding.QtWidgets import QWidget, QFileDialog, QFormLayout, QHBoxLayout, QPushButton, QLabel, QComboBox
from PyQt5.QtGui import QFont
from PyQt5.QtCore import pyqtSignal

class LoadRosbagPopup(QWidget):

    submitted = pyqtSignal(str, str, name='load_rosbag')

    def __init__(self):
        QWidget.__init__(self)
        self.resize(640, 480)
        # self.topic_edit = QLineEdit()
        # self.message_b_edit = QLineEdit()
        self.topic_dropdown = QComboBox()
        self.topic_dropdown.setEnabled(False)
        self.path = 'No file chosen'
        self.path_display = QLabel(
            text=self.path,
            font=QFont('Sans', 12)
        )
        self.file_button = QPushButton('File')
        self.cancel_button = QPushButton('Cancel')
        self.submit_button = QPushButton('Submit')
        self.submit_button.setEnabled(False)

        self.setLayout(QFormLayout())
        self.layout().addRow(self.file_button, self.path_display)
        self.layout().addRow('Topic', self.topic_dropdown)
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
        style_path = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MaterialDark.qss')
        with open(style_path, 'r') as qss:
            self.style = qss.read()
        self.setStyleSheet(self.style)

    def on_submit(self):
        self.submitted.emit( self.path, self.topic_dropdown.currentText() )
        self.close()

    def get_rosbag_filename(self):
        # Retrieve rosbag path string from file explorer window
        self.path = QFileDialog.getOpenFileName(self, 'Open a rosbag', '', 'Rosbag Files (*.bag)')[0]
        try:
            bag = rosbag.Bag(self.path)
        except:
            rospy.logerr('Unable to open file: %s', self.path)
        else:
            self.path_display.setText(self.path)
            # Check rosbag topics for PC2 data and build dropdown from those topics
            found_PC2_topic = False
            bag_type_and_topic_info = bag.get_type_and_topic_info()
            for index, topic in enumerate(bag_type_and_topic_info[1].values()):
                if 'PointCloud2' in topic[0]:
                    self.topic_dropdown.addItem(bag_type_and_topic_info[1].keys()[index])
                    found_PC2_topic = True
            # Enable dropdown selector and submit button if PC2 topic found
            if found_PC2_topic:
                self.topic_dropdown.setEnabled(True)
                self.submit_button.setEnabled(True)