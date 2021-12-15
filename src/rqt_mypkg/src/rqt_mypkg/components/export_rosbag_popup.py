import os
import rospy
import rosbag
import rospkg
from python_qt_binding.QtWidgets import QWidget, QProgressBar, QFileDialog, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QLineEdit
from PyQt5.QtGui import QFont
from PyQt5.QtCore import pyqtSignal

class ExportRosbagPopup(QWidget):

    submitted = pyqtSignal(str, str, name='export_rosbag')

    def __init__(self):
        QWidget.__init__(self)
        self.setWindowTitle('Export rosbag')
        self.resize(450, 230)

        # Create components
        self.annot_topic_name = QLineEdit('/annotations')
        self.path = 'No path chosen'
        self.path_display = QLabel(text=self.path, font=QFont('Sans', 10))
        self.file_button = QPushButton('Browse...')
        self.cancel_button = QPushButton('Cancel')
        self.submit_button = QPushButton('Submit')
        self.submit_button.setEnabled(False)
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)

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
        self.layout.addWidget(QLabel(text='Choose path:', font=QFont('Sans', 10)))
        self.layout.addWidget(file_widget)
        self.layout.addWidget(QLabel(''))
        self.layout.addWidget(QLabel(text='Annotation topic name:', font=QFont('Sans', 10)))
        self.layout.addWidget(self.annot_topic_name)
        self.layout.addWidget(QLabel(''))
        self.layout.addWidget(QLabel(''))
        self.layout.addWidget(self.progress_bar)
        self.layout.addWidget(QLabel(''))
        self.layout.addWidget(button_widget)
        self.setLayout(self.layout)

        # Load in styling for GUI
        style_path = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'dark.qss')
        with open(style_path, 'r') as qss:
            self.style = qss.read()
        self.setStyleSheet(self.style)

    def on_submit(self):
        self.submitted.emit( self.path, self.annot_topic_name.text() )
        self.close()

    def get_rosbag_filename(self):
        # Retrieve rosbag path string from file explorer window
        self.path = QFileDialog.getSaveFileName(self, 'Output Path', '/home/output.bag', 'Rosbag Files (*.bag)')[0]
        
        if not self.path:
            # User cancelled the file selection dialog, return
            self.submit_button.setEnabled(False)
            return
        elif self.path[-4:] != '.bag':
            self.path += '.bag'
        self.path_display.setText(self.path)
        self.submit_button.setEnabled(True)

    def set_progress_bar_range(self, max_progress):
        self.progress_bar.setRange(0, max_progress)

    def increment_progress_bar(self):
        self.progress_bar.setValue(self.progress_bar.value() + 1)
