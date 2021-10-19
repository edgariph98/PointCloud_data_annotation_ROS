from logging import error
import os
import rospy
import rosbag
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QPushButton, QSlider, QHBoxLayout
from python_qt_binding.QtCore import Qt
from sensor_msgs.msg import PointCloud2

# encapsulation of a bag player 
class BagPlayer(QWidget):

    # rvizTopic is the topic where pc2 messages are send to display on the rviz main frame
    # pointCloudTopic is the topic where we read messeges from our rosbag
    def __init__(self, rvizTopic, pointCloudTopic):
        super(BagPlayer, self).__init__()
        # used to publish pointcloud messages
        self.bagPublisher = None
        # the bag containing the messages and frames of Frame class as well as current frame number
        self.bag = None
        self.frames = []
        self.pointCloudTopic  = pointCloudTopic
        self.currentFrame = -1

        # ui components references
        self.slider = None
        self.nextButton = None
        self.prevButton = None

        # loading ui
        self.initUI()

        # setting up publisher 
        self.setUpPublisher(rvizTopic)

    # updates a bag with the given bag and frames
    def updateBag(self,_bag, _frames):
        self.bag = _bag
        self.frames  = _frames
        self.currentFrame = 0
        self.slider.setMaximum( len(self.frames)-1)
        rospy.loginfo("Rosbag updated, Number of Frames: {} ".format(len(self.frames)))
        # publish first frame
        self.publishFrame(self.currentFrame)

    # initializes ui with connected member callback functions
    def initUI(self):
        # main vertical layout
        layout = QVBoxLayout()

        # slider  attributes
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setTracking( True )
        self.slider.setMinimum( 1 )
        self.slider.setMaximum( len(self.frames))
        self.slider.valueChanged.connect(self.sliderOnChange)

        # button layout
        buttonBox = QHBoxLayout()
        self.nextButton, self.prevButton = QPushButton("Next"), QPushButton("Previous")
        self.nextButton.clicked.connect(self.playNext)
        self.prevButton.clicked.connect(self.playPrev)
        buttonBox.addWidget(self.prevButton)
        buttonBox.addWidget(self.nextButton)

        # main layout adding widgets
        layout.addWidget(self.slider)
        layout.addLayout(buttonBox)
        self.setLayout(layout)

    # setup publisher to publish point cloud messages
    def setUpPublisher(self, topic):
        try:
            self.bagPublisher = rospy.Publisher(topic,PointCloud2,queue_size=1)
        except error:
            rospy.logerr(error)
    
    # plays the next frame in the bag and sets the slider on that index
    def playPrev(self):
        if len(self.frames) > 1 and self.currentFrame > 0:
            self.currentFrame  -= 1
            self.slider.setValue(self.currentFrame)
    
    # plays the previous frame in the bag and sets the slider on that index
    def playNext(self):
        if len(self.frames) > 1 and self.currentFrame < len(self.frames) -1:
            self.currentFrame  += 1
            self.slider.setValue(self.currentFrame)

    # publish the message from a frame index in our frame list and publishing to the rviz topic to display it
    def publishFrame(self,frameNumber):
        # timestamp associated the message to read
        frameTimeStamp = self.frames[frameNumber].timestamp
        publishingMSG = None
        # getting pointcloud message 
        for topic, msg, t  in self.bag.read_messages(self.pointCloudTopic,start_time=frameTimeStamp,end_time=frameTimeStamp):
            publishingMSG = msg
        self.bagPublisher.publish(publishingMSG)
        rospy.loginfo("Current Frame {}, timestamp: {}".format(frameNumber,frameTimeStamp))

    # publishes a frame anytime the slider changes
    def sliderOnChange(self, newFrame):
        self.currentFrame = newFrame
        self.publishFrame(self.currentFrame)