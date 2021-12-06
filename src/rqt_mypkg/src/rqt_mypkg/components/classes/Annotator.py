import rospy
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from sensor_msgs.msg import PointCloud2
from .annotation import Annotation
from std_msgs.msg import ColorRGBA
from annotation_msgs.msg import Annotation as Annotation_msg
from std_msgs.msg import Bool as Bool_msg
from interactive_markers.menu_handler import MenuHandler
from PyQt5.QtCore import pyqtSignal, pyqtSlot,QObject

class Annotator(QObject):

    pending_annotation_marker = pyqtSignal(float, float, float, float, float, float, name='get_pending_annotation_marker')
    pending_annotation_points = pyqtSignal(name='get_pending_annotation_points') 
    rviz_cancelled_new_annotation = pyqtSignal(name='rviz_cancelled_new_annotation')
    annotation_details = pyqtSignal(str, str, str, float, float, float, float, float, float, name='annotation_details')
    delete_annotation_signal = pyqtSignal(str, name='delete_annotation')

    def __init__(self, _frames):
        super(Annotator, self).__init__()
        # publisher and subscriber to interact with the selected points publisher node
        try:
            # this publisher notifies the selection tool, that the annotation has been created
            self.annotationCreatedPublisher = rospy.Publisher(
                "/selection/annotation_completed", Marker, queue_size=1)

            # this subscriber of Annotation MSG gets the annotation message from the selected points tool
            self.annotation_selected_subscriber = rospy.Subscriber("/selection/annotation_selected_created",Annotation_msg,self._annotation_selected)
            # this subscriber us infromed with a Bool msg when a pending annotation has been removed from rviz
            self.annotation_selected_removed_subscriber =  rospy.Subscriber("/selection/annotation_selected_removed",Bool_msg,self._annotation_selected_removed)
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
        # menu 
        self.menu = self._create_menu_handler()
        self._update_annotation_ids()

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
                    newAnnotation.getInteractiveMarker(), self._annotation_marker_clicked)
                # applying changes to server and the menu for the interactive marker 
                self.menu.apply(self.server,id)
                self.server.applyChanges()
                # adding id to the set of annotation ids
                self.annotationIds.add(id)
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


    # clears all annotations from annotations from server at the current frame
    def _clearAnnotations(self):
        self.server.clear()
        self.server.applyChanges()

    # loading annotations for the frame index
    def loadAnnotations(self,
                        frameIndex  # int
                        ):
        self.remove_selection()
        self._clearAnnotations()
        self.currentFrame = frameIndex
        self._printLogMSG(
            "Loading Annotations")
        self.currentAnnotations = self.frames[self.currentFrame].annotations
        for annotation in self.currentAnnotations:
            self.menu.apply
            self.server.insert(
                annotation.getInteractiveMarker(), self._annotation_marker_clicked)
            self.menu.apply(self.server,annotation.getId())
        self.server.applyChanges()

    # print error msg

    def _printErrorMSG(self, msg):
        rospy.logerr("[Annotator] Current Frame [{}] ".format(self.currentFrame) + msg)

    # print log msg

    def _printLogMSG(self, msg):
        rospy.loginfo("[Annotator] Current Frame [{}] ".format(self.currentFrame) + msg)


    # remvoing selection from rviz 
    def remove_selection(self):
        emptyMarker = Marker()
        # when this publisher sends a message, it removes selection from rviz
        self.annotationCreatedPublisher.publish(emptyMarker)

    def _annotation_selected(
        self, 
        selected_annotation # Annotation_msg
        ):
        boundingBoxMarker = selected_annotation.bounding_box
        # we only use the bounding box marker if its not being deleted
        # if not boundingBoxMarker.action == Marker.DELETE:
        # scale and position of the annotation
        scale = boundingBoxMarker.scale
        position = boundingBoxMarker.pose.position
        # setting  the current selected annotation
        self.selected_annotation = selected_annotation
        self._printLogMSG("New Annotation selected, Total Points Selected: {}, Bounding Box Marker Scale : x [{}] y [{}] z [{}], Position  x [{}] y [{}] z [{}]".format(
            selected_annotation.num_points, scale.x, scale.y, scale.z, position.x, position.y, position.z))
        # if selection contains points send signal with marker details
        if selected_annotation.num_points > 0:
            self.pending_annotation_marker.emit(scale.x, scale.y, scale.z, position.x, position.y, position.z)

    # annotation callback when annotation selection has been removed in rviz
    def _annotation_selected_removed(self, _bool_msg):
        # updating that rviz has canclled an annotation
        self.rviz_cancelled_new_annotation.emit()
    
    # deletes an annotation given its annotation id on the current frame, returns boolean determining if deletion was successful
    @pyqtSlot(str, name='confirmed_delete_annotation')
    def delete_annotation(self, annotation_id, frame_num=None):
        annotations = self.currentAnnotations if not frame_num else self.frames[frame_num].annotations
        annotation_found = False
        annotation_index = 0
        for annotation in annotations:
            if annotation_id == annotation.getId():
                annotation_found = True
                break
            annotation_index +=1
        self._printLogMSG(str(annotation_found))
        # annotation found
        if annotation_found:
            # removing annotation from current set of annotations
            annotations.pop(annotation_index)
            #s removing annotation id from set of ids
            self.annotationIds.remove(annotation_id)
            # applying changes to the server
            self.server.erase(annotation_id)
            self.server.applyChanges()
            self._printLogMSG("Annotaion id: {} has been deleted!, # current annotations: {}, # total Annotations: {}".format(annotation_id,len(self.currentAnnotations), len(self.annotationIds)))
        else:
            self._printErrorMSG("Annotation  with id: {}, not found! deletetion failed...".format(annotation_id))
        return annotation_found

    def delete_annotation_group(self, annotation_group_id):
        for frame_index, frame in enumerate(self.frames):
            for annotation in frame.annotations:
                if annotation.group_id == annotation_group_id:
                    self.delete_annotation(annotation.id, frame_index)

    # creates a common menu for all markers from annotations
    def _create_menu_handler(self):
        menu = MenuHandler()
        menu.insert("Delete",callback=self._delete_menu_entry_clicked)
        return menu

    # callback function to processwhen right click on interactive from the menu
    def _delete_menu_entry_clicked(self, feedback):
        annotation_id = feedback.marker_name
        self._printLogMSG("Annotation ID: {}, Delete button clicked".format(feedback.marker_name))
        self.delete_annotation(annotation_id)
        # Update annotation list window
        self.delete_annotation_signal.emit(annotation_id)

    # call back function to process when a marker for an annotation has been clicked
    def _annotation_marker_clicked(self, feedback):
        annotation_id = feedback.marker_name
        clicked_annotation = self._find_annotation(annotation_id)

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK and clicked_annotation:
            scale = clicked_annotation.get_scale()
            position = clicked_annotation.intMarker.pose.position
            label = clicked_annotation.label if clicked_annotation.label else "NONE"
            group_id = clicked_annotation.group_id
            self._printLogMSG("Annotation ID: {}, Label:{}, groupId: {}, scale x:{}, y:{}, z:{}, Position x:{}, y:{}, z:{} has been clicked!".format(
                annotation_id, label, group_id, scale.x, scale.y, scale.z, position.x, position.y, position.z))
            # Send annotation details to the details window
            self.annotation_details.emit(annotation_id, label, group_id, scale.x, scale.y, scale.z, position.x, position.y, position.z)
    
    # returns the annotation object based on the ID on the current frame, if not found returns None
    def _find_annotation(self, annotation_id):
        for annotation in self.currentAnnotations:
            if annotation_id == annotation.getId():
                return annotation
        return None
    # when the annotator is initilized we check for any existing annotation in the frames and add them in the set for ids
    def _update_annotation_ids(self):
        self.annotationIds = set()
        for _frame in self.frames:
            for _annotation in _frame.annotations:
                self.annotationIds.add(_annotation.getId())
    
    # updating frames
    def update_frames(self,_frames):
        self.frames = _frames
        self._update_annotation_ids()
        self._clearAnnotations()


