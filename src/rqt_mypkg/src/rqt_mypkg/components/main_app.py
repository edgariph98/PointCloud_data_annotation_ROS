import os
import rospy
import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QPushButton
import rviz 
# Main App widget to be imported in RQT Plugin
class MainApp(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        # RVIZ Visualization Frame Loading
        self.frame = rviz.VisualizationFrame()
        self.frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        filePath = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'vizualization_frame.config.rviz')
        reader.readFile( config, filePath)
        self.frame.load( config )
        ####################################

        # Loading Box Vertical layout
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.frame)
        self.setLayout(self.layout)

