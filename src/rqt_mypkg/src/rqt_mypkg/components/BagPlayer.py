from pickle import NONE
from datetime import datetime
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QPushButton, QSlider, QHBoxLayout, QLineEdit, QLabel
from python_qt_binding.QtGui import QIntValidator
from python_qt_binding.QtCore import Qt
from sensor_msgs.msg import PointCloud2
import rospy
from PyQt5.QtCore import pyqtSignal

# encapsulation of a bag player
class BagPlayer(QWidget):

    changed_frame = pyqtSignal(int, name='changed_frame')

    # rvizTopic is the topic where pc2 messages are send to display on the rviz main frame
    # pointCloudTopic is the topic where we read messeges from our rosbag
    def __init__(self, rvizTopic):
        super(BagPlayer, self).__init__()
        # used to publish pointcloud messages
        self.bagPublisher = None
        # the bag containing the messages and frames of Frame class as well as current frame number
        self.bag = None
        self.frames = []
        self.pointCloudTopic = None
        self.currentFrame = -1
        self.annotator = None
        # ui components references
        self.timestamp = None
        self.frameInput = None
        self.frameLabel = None
        self.slider = None
        self.nextButton = None
        self.prevButton = None

        # loading ui
        self.initUI()

        # setting up publisher
        self.setUpPublisher(rvizTopic)

    # updates a bag with the given bag and frames
    def updateBag(self, _pointCloudTopic, _bag, _frames, _annotator):
        self.pointCloudTopic = _pointCloudTopic
        self.bag = _bag
        self.frames = _frames
        self.annotator = _annotator
        self.currentFrame = 0
        self.slider.setMaximum(len(self.frames)-1)
        self.frameInput.setValidator(QIntValidator(0, len(self.frames) - 1))
        self.frameLabel.setText("/{}".format(len(self.frames)-1))
        rospy.loginfo(
            "Rosbag updated, Number of Frames: {} ".format(len(self.frames)))
        # publish first frame
        self.slider.setValue(self.currentFrame)
        self.publishFrame(self.currentFrame)
        self.frameInput.setEnabled(True)

    # initializes ui with connected member callback functions
    def initUI(self):
        # main vertical layout
        layout = QVBoxLayout()

        # text input/timestamp layout        
        self.timestamp = QLabel("Timestamp: ")
        self.frameInput = QLineEdit()
        self.frameInput.setAlignment(Qt.AlignRight)
        self.frameInput.setStyleSheet("width: 65px")
        self.frameInput.setEnabled(False)
        self.frameInput.returnPressed.connect(self.inputOnChange)
        self.frameLabel = QLabel("/" + str(len(self.frames) - 1))

        frameLayout = QHBoxLayout()
        frameLayout.addWidget(QLabel("Frame"))
        frameLayout.addWidget(self.frameInput)        
        frameLayout.addWidget(self.frameLabel, 1)
        frameNumberWidget = QWidget()
        frameNumberWidget.setLayout(frameLayout)

        # slider  attributes
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setTracking(True)
        self.slider.setMinimum(1)
        self.slider.setMaximum(len(self.frames))
        self.slider.valueChanged.connect(self.sliderOnChange)

        # Create buttons and build the bottom row's layout
        bottomRowLayout = QHBoxLayout()
        self.nextButton, self.prevButton = QPushButton(
            "Next"), QPushButton("Previous")
        self.nextButton.clicked.connect(self.playNext)
        self.prevButton.clicked.connect(self.playPrev)
        bottomRowLayout.addWidget(self.timestamp, 1)
        bottomRowLayout.addWidget(self.prevButton, 1)
        bottomRowLayout.addWidget(self.nextButton, 1)
        bottomRowLayout.addWidget(frameNumberWidget, 1, Qt.AlignRight)

        # main layout adding widgets
        layout.addWidget(self.slider)
        layout.addLayout(bottomRowLayout)
        self.setLayout(layout)

    # setup publisher to publish point cloud messages
    def setUpPublisher(self, topic):
        try:
            self.bagPublisher = rospy.Publisher(
                topic, PointCloud2, queue_size=1)
        except Exception as error:
            rospy.logerr(
                "unable to set publisher for rosbag player because: {}".format(error))

    # plays the next frame in the bag and sets the slider on that index
    def playPrev(self):
        if len(self.frames) > 1 and self.currentFrame > 0:
            self.currentFrame -= 1
            self.slider.setValue(self.currentFrame)
            self.frameInput.setText("{}".format(self.currentFrame))

    # plays the previous frame in the bag and sets the slider on that index
    def playNext(self):
        if len(self.frames) > 1 and self.currentFrame < len(self.frames) - 1:
            self.currentFrame += 1
            self.slider.setValue(self.currentFrame)
            self.frameInput.setText("{}".format(self.currentFrame))

    # publish the message from a frame index in our frame list and publishing to the rviz topic to display it
    def publishFrame(self, frameNumber):
        # timestamp associated the message to read
        frameTimeStamp = self.frames[frameNumber].timestamp
        self.timestamp.setText("Timestamp: {}".format(datetime.fromtimestamp(frameTimeStamp.to_sec())))
        self.frameInput.setText("{}".format(frameNumber))
        publishingMSG = None
        # getting pointcloud message
        for topic, msg, t in self.bag.read_messages(self.pointCloudTopic, start_time=frameTimeStamp, end_time=frameTimeStamp):
            publishingMSG = msg
        self.bagPublisher.publish(publishingMSG)
        rospy.loginfo("Current Frame {}, timestamp: {}".format(
            frameNumber, frameTimeStamp))
        self.annotator.loadAnnotations(frameNumber)
        # Send new frame number to annotation list window
        self.changed_frame.emit(frameNumber)

    def inputOnChange(self):
        self.currentFrame = int(self.frameInput.text())
        self.publishFrame(self.currentFrame)
        self.slider.setValue(self.currentFrame)

    # publishes a frame anytime the slider changes
    def sliderOnChange(self, newFrame):
        self.currentFrame = newFrame
        self.publishFrame(self.currentFrame)
        self.frameInput.setText("{}".format(self.currentFrame))
