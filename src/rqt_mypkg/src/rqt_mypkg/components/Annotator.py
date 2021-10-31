from time import time
import rospy
from interactive_markers.interactive_marker_server import *
from rospy.core import loginfo
from visualization_msgs.msg import *
from sensor_msgs.msg import PointCloud2
from classes import AnnotationGroup
from python_qt_binding.QtWidgets import QWidget
from classes import Annotation
import rviz
import time
from std_msgs.msg import ColorRGBA


class Annotator():

    def __init__(self, _rviz_frame, _frames):
        # super(Annotator, self).__init__()
        # subscriber to bounding box publisher topic from selection tool
        try:
            self.boundingBoxSubscriber = rospy.Subscriber(
                "/visualization_marker", Marker, self._addAnnotation)
        except Exception as e:
            self._printErrorMSG("{}".format(e))
        # _rviz_frame reference
        self.rviz_frame = _rviz_frame
        # unique ids of all annotations across all frames
        self.annotationIds = set()
        # manager used change the tool
        self.tool_manager = self.rviz_frame.getManager().getToolManager()
        # selection tool to grab input from screen
        self.selectPointsTool = self.tool_manager.getTool(1)
        self._printLogMSG("Tools:  {}, {}".format(
            self.tool_manager.getDefaultTool().getName(), self.selectPointsTool.getName()))
        # create an interactive marker server on the topic namespace simple_marker that publishes shapes
        self.server = InteractiveMarkerServer("simple_marker")
        self.frames = _frames
        # each empty list is list annotation corresponding to the frame being showed
        self.framesAnnotations = [[] for i in range(
            len(self.frames))]  # List [ List [Annotation] ]
        self.currentFrame = 0
        # current set of annotations -> List[ Annotation ]
        self.currentAnnotations = self.framesAnnotations[self.currentFrame]
        # adding object used when in ADDING-MODE
        self.addingObject = {
            "label": "",
            "group": "",
            "id": "",
            "color": None,
            "addingMode": False
        }

    # sets the mode annotation to adding and creates and accessible object containing the information needed to created the Annotation
    def createAnnotation(
        self,
        groupName,  # str
        labelName,  # str
        id,         # str
        color       # ColorRGBA
    ):
        if id in self.annotationIds:
            self._printErrorMSG(
                "Annotation with id \"{}\" already exists, provide a unique id".format(id))
            return False
        else:
            self.addingObject["label"] = labelName
            self.addingObject["group"] = groupName
            self.addingObject["id"] = id
            self.addingObject["color"] = color
            self.addingObject["addingMode"] = True
            # changing to selecPointsTool to create the the box
            self.rviz_frame.getManager().getToolManager().setCurrentTool(self.selectPointsTool)
            self._printLogMSG("Entering Adding Mode for Annotation with id :{}, Label: {}, Group: {}, Color Values R: {}, G: {}, B: {}, A: {}".format(
                id, labelName, groupName, color.r, color.g, color.b, color.a))

    # when annotator is in adding mode and the user makes a selection this adds marker to the server of the new annotation
    # this is the callback used when the selection tool is done gathering the point cloud data from the window
    def _addAnnotation(self,
                       boundingBoxMarker  # Marker
                       ):
        # the adding object parameters
        addingMode = self.addingObject["addingMode"]
        group = self.addingObject["group"]
        label = self.addingObject["label"]
        id = self.addingObject["id"]
        color = self.addingObject["color"]
        # if in addingMode we create a new annotation object and added to the current annotation list 
        if addingMode:
            newAnnotation = Annotation(
                id, label, group, boundingBoxMarker, color)
            self.currentAnnotations.append(newAnnotation)
            # inserting the marker of the new annotation in the server
            self.server.insert(
                newAnnotation.getInteractiveMarker(), self.processFeedback)
            self.server.applyChanges()

            self.addingObject["addingMode"] = False
            self._printLogMSG("Current Frame [{}], new annotation with Id: {}, Label \"{}\", Group \"{}\", Color Values R: {}, G: {}, B: {}, A: {}".format(
                self.currentFrame, id, label, group, color.r, color.g, color.b, color.a))
        else:
            self._printLogMSG(
                "Current Frame [{}], Adding mode is turned off".format(self.currentFrame))

    # TODO
    # processing feedback for each marker
    def processFeedback(self, feedback):
        p = feedback.pose.position
        print(feedback.marker_name + " is now at " +
              str(p.x) + ", " + str(p.y) + ", " + str(p.z))

    # clears all annotations from annotations from server at the current frame
    def _clearAnnotations(self):
        self.server.clear()
        self.server.applyChanges()

    # loading annotations for the frame index
    def loadAnnotations(self,
                        frameIndex  # int
                        ):
        self._clearAnnotations()
        self._printLogMSG(
            "Current Frame [{}], Loading Annotations".format(frameIndex))
        self.currentFrame = frameIndex
        self.currentAnnotations = self.framesAnnotations[self.currentFrame]
        for annotation in self.currentAnnotations:
            self.server.insert(
                annotation.getInteractiveMarker(), self.processFeedback)
        self.server.applyChanges()

    # print error msg

    def _printErrorMSG(self, msg):
        rospy.logerr("[Annotator] " + msg)

    # print log msg

    def _printLogMSG(self, msg):
        rospy.loginfo("[Annotator] " + msg)
