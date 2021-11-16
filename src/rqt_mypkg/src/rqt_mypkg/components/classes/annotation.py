import uuid
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from std_msgs.msg import ColorRGBA

class Annotation:
    def __init__(self,
                 _id,               
                 _label,
                 _group_id,
                 _marker,
                 _color,
                 _pc2_msg
                 ):

        # Unique id for an annotation
        self.id = _id
        # label for the annotation
        self.label = _label
        # the annotation group id
        self.group_id = _group_id
        # regular Marker msg
        self.marker = _marker
        # interactive marker
        self.intMarker = self._createInteractiveMarker( _color)
        # the pointcloud 2 data catptured by the annotation
        self.captured_point_cloud = _pc2_msg


    
    # creates the marker box with all the shapes with given id
    def _createInteractiveMarker(self, color):
        intMarker = InteractiveMarker()
        intMarker.name = self.id
        intMarker.description = "id:{},\nGroup: {},\nLabel: {},\n".format(
            self.id, self.group_id, self.label)
        intMarker.header.frame_id = self.marker.header.frame_id
        intMarker.pose = self.marker.pose
        # changing color of regular marker
        self.marker.color = color
        intMarker.controls.extend(
            self._createInteractiveMarkerControls(self.marker))
        return intMarker

    # creates the controls for the marker
    def _createInteractiveMarkerControls(self, marker):
        container_control = InteractiveMarkerControl()
        container_control.interaction_mode  = InteractiveMarkerControl.BUTTON
        container_control.always_visible = True
        container_control.markers = [marker]
        #move marker along x axis control
        move_x_control = InteractiveMarkerControl()
        move_x_control.name = "move_x"
        move_x_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        move_x_control.always_visible = True
        move_x_control.orientation.w = marker.pose.orientation.w
        move_x_control.orientation.x = marker.pose.orientation.x + marker.scale.x
        # controls to add
        controls = [container_control]
        return controls

    # returns label of the  annotation 
    def getName(self):
        return self.label

    # returns the id of the annotation
    def getId(self):
        return self.id
    

    # returns the interactive marker
    def getInteractiveMarker(self):
        return self.intMarker

    # returns scale object from the bounding box marker
    def get_scale(self):
        return self.intMarker.controls[0].markers[0].scale
    
