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
from PyQt5.QtCore import pyqtSignal, pyqtSlot, QObject
from ..auxiliary_functions import get_annotation_group_by_id

class Annotator(QObject):

    pending_annotation_marker = pyqtSignal(float, float, float, float, float, float, name='get_pending_annotation_marker')
    pending_annotation_points = pyqtSignal(str, name='get_pending_annotation_points')
    rviz_cancelled_new_annotation = pyqtSignal(name='rviz_cancelled_new_annotation')

    def __init__(self, _frames, _annotation_groups):
        super(Annotator, self).__init__()
        # publisher and subscriber to interact with the selected points publisher node
        try:
            # this publisher gets the bounding  bounding Box marker of the selection
            self.boundingBoxSubscriber = rospy.Subscriber(
                "/visualization_marker", Marker, self._getSelection)
            # this publisher notifies the selection tool, that the annotation has been created
            self.annotationCreatedPublisher = rospy.Publisher(
                "/annotation_created", Marker, queue_size=1)
        except Exception as e:
            self._printErrorMSG("{}".format(e))
        # unique ids of all annotations across all frames
        self.annotationIds = set()
        # create an interactive marker server on the topic namespace simple_marker that publishes shapes
        self.server = InteractiveMarkerServer("simple_marker")
        self.annotation_groups = _annotation_groups
        self.frames = _frames
        self.currentFrame = 0
        # current set of annotations -> List[ Annotation ]
        self.currentAnnotations = self.frames[self.currentFrame].annotations
        # Marker, containing the current bounding Box selected from the user
        self.current_bounding_box_selection = None

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
            if self.current_bounding_box_selection:
                # creating new annotation
                newAnnotation = Annotation(
                    id, labelName, groupName, self.current_bounding_box_selection, color)
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
                self._printLogMSG("New Annotation Created id :{}, Label: {}, Group: {}, Color Values R: {}, G: {}, B: {}, A: {}".format(
                    id, labelName, groupName, color.r, color.g, color.b, color.a))
                # proper creation of annotation
                success = True
            # no selection has been made by the user
            else:
                self._printErrorMSG("No selection has been made")

            return success

    # this is the callback used when the selection tool is done gathering the point cloud data from the window
    def _getSelection(self,
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
            # Send signal with marker details
            self.pending_annotation_marker.emit(scale.x, scale.y, scale.z, position.x, position.y, position.z)
        else: 
            pass
            # send cancel signal to detials
            

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


    @pyqtSlot(str, str, str, name='confirm_annotation')
    def get_confirmed_annotation(self, label, annotation_id, group_id):
        # Edgar:
        # I can't send a color using slots/signals. 
        # Instead the Annotator now has access to the annotation_group list and can use the group_id
        group_name = get_annotation_group_by_id(self.annotation_groups, group_id).name
        self._printLogMSG("Recieved from annotation_details_window id :{}, Label: {}, Group id: {}, Group Name: {}".format(
            annotation_id, label, group_id, group_name))
    
    @pyqtSlot(name='cancelled_new_annotation')
    def cancelled_new_annotation(self):
        #  Todo
        # remove the marker
        # reset reviz window state to whatever it should be
        print("Annotation just got cancelled")
        self.rviz_cancelled_new_annotation.emit()

    # Todo
    # execute 'self.rviz_cancelled_new_annotation.emit()' when the user deletes a marker using rviz window
    
    

