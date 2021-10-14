import os
import rospy
import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QPushButton
import rviz 
from classes import Frame
import rosbag
# Main App widget to be imported in RQT Plugin
class MainApp(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        # frames of the rosbag
        self.frames = []
        # rosbag object
        self.bag = None
    
        # RVIZ Visualization Frame Loading
        self.rviz_frame = rviz.VisualizationFrame()
        self.rviz_frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        filePath = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'vizualization_frame.config.rviz')
        reader.readFile( config, filePath)
        self.rviz_frame.load( config )
        ####################################

        # Loading Box Vertical layout
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.rviz_frame)
        self.setLayout(self.layout)

    # opening a rosbag and loading frames
    def load_rosbag(self, path, topic_name):
        try:
            self.bag = rosbag.Bag(path)
        except:
            rospy.logerr('Unable to open file: %s', path)
        else:
            if topic_name not in self.bag.get_type_and_topic_info()[1].keys():
                rospy.logerr('Topic \'%s\' not found in rosbag', topic_name)
                self.bag.close()
                return
            # Create empty list of Frames for each message
            self.frames = [Frame]*self.bag.get_message_count(topic_name)
            # Iterate through msgs and populate the empty list
            index = 0
            for topic, msg, t in self.bag.read_messages(topics=[topic_name]):
                frame = Frame(t)
                self.frames[index] = frame
                index += 1
            # Load first frame to viewer here
            self.bag.close()



