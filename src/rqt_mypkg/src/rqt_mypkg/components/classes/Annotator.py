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
from PyQt5.QtCore import pyqtSignal, pyqtSlot, QObject

class Annotator(QObject):

    pending_annotation_marker = pyqtSignal(float, float, float, float, float, float, name='get_pending_annotation_marker')
    pending_annotation_points = pyqtSignal(name='get_pending_annotation_points') 
    rviz_cancelled_new_annotation = pyqtSignal(name='rviz_cancelled_new_annotation')

    def __init__(self, _frames):
        super(Annotator, self).__init__()
        # publisher and subscriber to interact with the selected points publisher node
        try:
            # this publisher notifies the selection tool, that the annotation has been created
            self.annotationCreatedPublisher = rospy.Publisher(
                "/selection/annotation_completed", Marker, queue_size=1)

            # this subscriber of Annotation MSG gets the annotation message from the selected points tool
            self.annotation_selection_subscriber = rospy.Subscriber("/selection/annotation",Annotation_msg,self._get_annotation_selection)
        except Exception as e:
            self._printErrorMSG("{}".format(e))
        # unique ids of all annotations across all frames
        self.annotationIds = set()
        # create an interactive marker server on the topic namespace simple_marker that publishes shapes
        self.server = InteractiveMarkerServer("simple_marker")
        self.frames = _frames
        self.currentFrame = 0
        # current set of annotations -> List[ Annotation ]
        self.currentAnnotations = self.frames[self.currentFrame].annotations
        # the annotation msg selection sent from rviz whene selecting a set of points 
        self.selected_annotation = None

    # sets the mode annotation to adding and creates and accessible object containing the information needed to created the Annotation
    # returns
    def createAnnotation(
        self,
        group_id,   # str
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
            if self.selected_annotation:
                # the annotation objects 
                annotationSelection  = Annotation_msg()
                annotationSelection.bounding_box = self.selected_annotation.bounding_box
                annotationSelection.captured_point_cloud = self.selected_annotation.captured_point_cloud
                annotationSelection.num_points = self.selected_annotation.num_points
                # creating new annotation
                newAnnotation = Annotation(
                    id, labelName, group_id, annotationSelection.bounding_box, color, annotationSelection.captured_point_cloud)
                # adding new annotation to the current set of annotations
                self.currentAnnotations.append(newAnnotation)
                # inserting the marker of the new annotation in the server
                self.server.insert(
                    newAnnotation.getInteractiveMarker(), self.processFeedback)
                self.server.applyChanges()
                # removing  selection from rviz
                self.remove_selection()
                self.selected_annotation = None
                self._printLogMSG("New Annotation Created id :{}, Label: {}, Group: {}, total PC2 points: {}, Color Values R: {}, G: {}, B: {}, A: {}".format(
                    id, labelName, group_id,annotationSelection.num_points, color.r, color.g, color.b, color.a))
                # proper creation of annotation
                success = True
            # no selection has been made by the user
            else:
                self._printErrorMSG("No selection has been made")

            return success
            
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
        self.currentAnnotations = self.frames[self.currentFrame].annotations
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


    # remvoing selection from rviz 
    def remove_selection(self):
        emptyMarker = Marker()
        # when this publisher sends a message, it removes selection from rviz
        self.annotationCreatedPublisher.publish(emptyMarker)

    def _get_annotation_selection(self, selected_annotation):
        boundingBoxMarker = selected_annotation.bounding_box
        # we only use the bounding box marker if its not being deleted
        if not boundingBoxMarker.action == Marker.DELETE:
            scale = boundingBoxMarker.scale
            position = boundingBoxMarker.pose.position
            # setting  the current selected annotation
            self.selected_annotation = selected_annotation
            self._printLogMSG("New Annotation selected, Total Points Selected: {}, Bounding Box Marker Scale : x [{}] y [{}] z [{}], Position  x [{}] y [{}] z [{}]".format(
                selected_annotation.num_points, scale.x, scale.y, scale.z, position.x, position.y, position.z))
            # Send signal with marker details
            self.pending_annotation_marker.emit(scale.x, scale.y, scale.z, position.x, position.y, position.z)
        else:
            self.rviz_cancelled_new_annotation.emit()
    # Todo
    # 1. Emit signal for new annotation point info 'self.pending_annotation_points.emit(values whatever they are)'

    

