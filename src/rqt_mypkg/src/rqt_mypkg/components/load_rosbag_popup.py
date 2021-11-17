import os
import rospy
import rosbag
import rospkg
from python_qt_binding.QtWidgets import QWidget, QFileDialog, QHBoxLayout, QVBoxLayout, QPushButton, QLabel, QComboBox
from PyQt5.QtGui import QFont
from PyQt5.QtCore import pyqtSignal

class LoadRosbagPopup(QWidget):

    submitted = pyqtSignal(str, str, str, str, name='load_rosbag')

    def __init__(self):
        QWidget.__init__(self)
        self.setWindowTitle('Load rosbag')
        self.resize(450, 230)

        # Create components
        self.topic_dropdown = QComboBox()
        self.topic_dropdown.setEnabled(False)
        self.annot_dropdown = QComboBox()
        self.annot_dropdown.setEnabled(False)
        self.group_dropdown = QComboBox()
        self.group_dropdown.setEnabled(False)
        self.path = 'No file chosen'
        self.path_display = QLabel(text=self.path, font=QFont('Sans', 10))
        self.file_button = QPushButton('Browse...')
        self.file_button.setFixedSize(125, 25)
        self.cancel_button = QPushButton('Cancel')
        self.submit_button = QPushButton('Submit')
        self.submit_button.setEnabled(False)

        # Connect buttons to signals and functions
        self.file_button.clicked.connect(self.get_rosbag_filename)
        self.submit_button.clicked.connect(self.on_submit)
        self.cancel_button.clicked.connect(self.close)
        
        # Create file select row widget
        file_widget = QWidget()
        file_widget.setLayout(QHBoxLayout())
        file_widget.layout().addWidget(self.file_button)
        file_widget.layout().addWidget(self.path_display)

        # Create button row widget
        button_widget = QWidget()
        button_widget.setLayout(QHBoxLayout())
        button_widget.layout().addWidget(self.cancel_button)
        button_widget.layout().addWidget(self.submit_button)

        # Add widgets to main layout
        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(10, 0, 10, 0)
        self.layout.addWidget(QLabel(''))
        self.layout.addWidget(QLabel(text='Choose file:', font=QFont('Sans', 10)))
        self.layout.addWidget(file_widget)
        self.layout.addWidget(QLabel(''))
        self.layout.addWidget(QLabel(''))
        self.layout.addWidget(QLabel(text='Choose LiDAR topic:', font=QFont('Sans', 10)))
        self.layout.addWidget(self.topic_dropdown)
        self.layout.addWidget(QLabel(text='Choose annotation topic:', font=QFont('Sans', 10)))
        self.layout.addWidget(self.annot_dropdown)
        self.layout.addWidget(QLabel(text='Choose group topic:', font=QFont('Sans', 10)))
        self.layout.addWidget(self.group_dropdown)
        self.layout.addWidget(QLabel(''))
        self.layout.addWidget(QLabel(''))
        self.layout.addWidget(button_widget)
        self.setLayout(self.layout)

        # Load in styling for GUI
        style_path = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'dark.qss')
        with open(style_path, 'r') as qss:
            self.style = qss.read()
        self.setStyleSheet(self.style)

    def on_submit(self):
        self.submitted.emit( self.path, self.topic_dropdown.currentText(), self.annot_dropdown.currentText(), self.group_dropdown.currentText() )
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
            found_annotation_topic = False
            found_group_topic = False
            bag_type_and_topic_info = bag.get_type_and_topic_info()
            for index, topic in enumerate(bag_type_and_topic_info[1].values()):
                if 'PointCloud2' in topic[0]:
                    self.topic_dropdown.addItem(bag_type_and_topic_info[1].keys()[index])
                    found_PC2_topic = True
                elif 'frame' in topic[0]:
                    self.annot_dropdown.addItem(bag_type_and_topic_info[1].keys()[index])
                    found_annotation_topic = True
                elif 'group' in topic[0]:
                    self.group_dropdown.addItem(bag_type_and_topic_info[1].keys()[index])
                    found_group_topic = True
                else:
                    rospy.loginfo(topic[0])   
            if found_annotation_topic:
                self.annot_dropdown.setEnabled(True)
                self.submit_button.setEnabled(True)
            # Enable dropdown selector and submit button if PC2 topic found
            if found_PC2_topic:
                self.topic_dropdown.setEnabled(True)
                self.submit_button.setEnabled(True)
        
