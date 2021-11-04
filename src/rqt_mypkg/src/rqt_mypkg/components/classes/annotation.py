import uuid
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from std_msgs.msg import ColorRGBA
from std_msgs.msg import ColorRGBA

class Annotation:
    def __init__(self,
                 _label,
                 _group,
                 _marker,
                 _color
                 ):
        # TODO
        # self.label = _label
        # self.group = _group
        # self.x_min = _x_min
        # self.x_max = _x_max
        # self.y_min = _y_min
        # self.y_max = _y_max
        # self.z_min = _z_min
        # self.z_max = _z_max

        # Unique id for an annotation, then put that id into field for PC2 points
        self.id = str(uuid.uuid4())
        # label for the annotation
        self.label = _label
        # the annotation group
        self.group = _group
        self.intMarker = self._createInteractiveMarker(_marker, _color)

    
    # creates the marker box with all the shapes with given id
    def _createInteractiveMarker(self, marker, color):
        intMarker = InteractiveMarker()
        intMarker.name = self.id
        intMarker.description = "id:{},\nGroup: {},\nLabel: {},\n".format(
            self.id, self.group, self.label)
        intMarker.header.frame_id = marker.header.frame_id
        intMarker.pose = marker.pose
        marker.color = ColorRGBA(.5, .5, .5, .6)
        intMarker.controls.extend(
            self._createInteractiveMarkerControls(marker))
        return intMarker

    # creates the controls for the marker
    def _createInteractiveMarkerControls(self, marker):
        container_control = InteractiveMarkerControl()
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
