from time import time
import rospy
from interactive_markers.interactive_marker_server import *
from rospy.core import loginfo
from visualization_msgs.msg import *
from sensor_msgs.msg import PointCloud2
from .annotation import Annotation
import rviz
import time
from std_msgs.msg import ColorRGBA
from annotation_msgs.msg import Annotation as Annotation_msg



class Annotator():

    def __init__(self, _frames):
        # publisher and subscriber to interact with the selected points publisher node
        try:
            # this publisher gets the bounding  bounding Box marker of the selection
            self.boundingBoxSubscriber = rospy.Subscriber(
                "/visualization_marker", Marker, self._get_marker_selection)
            # this publisher notifies the selection tool, that the annotation has been created
            self.annotationCreatedPublisher = rospy.Publisher(
                "/annotation_created", Marker, queue_size=1)
            # this subscriber gets the point cloud data selected by the user
            self.annotationPC2Subscriber = rospy.Subscriber("/rviz_selected_points",PointCloud2,self._get_pc2_selection)
        except Exception as e:
            self._printErrorMSG("{}".format(e))
        # unique ids of all annotations across all frames
        self.annotationIds = set()
        # create an interactive marker server on the topic namespace simple_marker that publishes shapes
        self.server = InteractiveMarkerServer("simple_marker")
        self.frames = _frames
        # each empty list is list annotation corresponding to the frame being showed
        self.framesAnnotations = [[] for i in range(
            len(self.frames))]  # List [ List [Annotation] ]
        self.currentFrame = 0
        # current set of annotations -> List[ Annotation ]
        self.currentAnnotations = self.framesAnnotations[self.currentFrame]
        # Marker, containing the current bounding Box selected from the user
        self.current_bounding_box_selection = None
        self.current_point_cloud2_selection = None

    # sets the mode annotation to adding and creates and accessible object containing the information needed to created the Annotation
    # returns
    def createAnnotation(
        self,
        groupName,  # str
        labelName,  # str
        id,         # str
        color       # ColorRGBA
    ):
        success = False
        if id in self.annotationIds:
            self._printErrorMSG(
                "Annotation with id \"{}\" already exists, provide a unique id".format(id))
        else:
            # the user has made a selection
            if self.current_bounding_box_selection and self.current_point_cloud2_selection:
                # creating new annotation
                newAnnotation = Annotation(
                    id, labelName, groupName, self.current_bounding_box_selection, color,self.current_point_cloud2_selection)
                # adding new annotation to the current set of annotations
                self.currentAnnotations.append(newAnnotation)
                # inserting the marker of the new annotation in the server
                self.server.insert(
                    newAnnotation.getInteractiveMarker(), self.processFeedback)
                self.server.applyChanges()
                # resetting the selection to None
                self.annotationCreatedPublisher.publish(
                    self.current_bounding_box_selection)
                self.current_bounding_box_selection = None
                totalPoints  = self.current_point_cloud2_selection.row_step / self.current_point_cloud2_selection.point_step
                self._printLogMSG("New Annotation Created id :{}, Label: {}, Group: {}, total PC2 points: {}, Color Values R: {}, G: {}, B: {}, A: {}".format(
                    id, labelName, groupName,totalPoints, color.r, color.g, color.b, color.a))
                # proper creation of annotation
                success = True
            # no selection has been made by the user
            else:
                self._printErrorMSG("No selection has been made")

            return success
    # callback that gets the selected points from the selection tool
    def _get_pc2_selection(self,pc2_msg):
        self.current_point_cloud2_selection = pc2_msg
        totalPoints = pc2_msg.row_step / pc2_msg.point_step
        self._printLogMSG("New point cloud selection added, total points: {}".format(totalPoints))
    # this is the callback used when the selection tool is done gathering the point cloud data from the window
    # it gets the bounding box of the selected points 
    def _get_marker_selection(self,
                      boundingBoxMarker  # Marker
                      ):
        # we only use the bounding box marker if its not being deleted
        if not boundingBoxMarker.action == Marker.DELETE:
            scale = boundingBoxMarker.scale
            position = boundingBoxMarker.pose.position
            # setting  the selection from the user to the bounding Box marker
            self.current_bounding_box_selection = boundingBoxMarker
            self._printLogMSG("New Bounding box selection, Marker Scale : x [{}] y [{}] z [{}], Position  x [{}] y [{}] z [{}]".format(
                scale.x, scale.y, scale.z, position.x, position.y, position.z))

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
