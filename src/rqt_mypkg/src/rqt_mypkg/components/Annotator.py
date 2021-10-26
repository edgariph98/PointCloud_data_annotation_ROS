
import rospy
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from python_qt_binding.QtWidgets import QWidget, QPushButton, QVBoxLayout
from sensor_msgs.msg import PointCloud2
class Annotator(QWidget):

    def __init__(self, _rviz_frame):
        super(Annotator, self).__init__()
        # subscriber to bounding box publisher topic from selection tool
        try:
            self.boundingBoxSubscriber = rospy.Subscriber("/visualization_marker",Marker, self.addMarker)
        except Exception as e:
            rospy.logerr(e)
        # _rviz_frame reference
        self.rviz_frame = _rviz_frame
        # create an interactive marker server on the topic namespace simple_marker
        self.server = InteractiveMarkerServer("simple_marker")
        self.annotations = []
        self.initUI()


    def initUI(self):
        self.moveButton  = QPushButton("add marker")
        self.moveButton.clicked.connect(self.addMarker)
        layout = QVBoxLayout()
        layout.addWidget(self.moveButton)
        self.setLayout(layout)



    def addMarker(self,boundingBoxMarker):
        # creating new marker with same properties
        newMarker = boundingBoxMarker
        # changing color
        newMarker.color.r = 0.0
        newMarker.color.g = 0.5
        newMarker.color.b = 0.5
        newMarker.color.a = 0.5
        # adding marker to list
        self.annotations.append(newMarker)
        # creating 
        newIntMarker = InteractiveMarker()
        newIntMarker.pose = newMarker.pose
        newIntMarker.header.frame_id = boundingBoxMarker.header.frame_id
        newIntMarker.header.stamp = rospy.get_rostime()
        newIntMarker.name = "newMarker"+str(len(self.annotations))
        newIntMarker.description = newIntMarker.name
        # adding controls with new marker
        newIntMarker.controls.extend(self.boundingBoxControls(newMarker))

        self.server.insert(newIntMarker,self.processFeedback)
        self.server.applyChanges()


    def processFeedback(self, feedback ):
        p = feedback.pose.position
        print(feedback.marker_name + " is now at " +
              str(p.x) + ", " + str(p.y) + ", " + str(p.z))
    

    def boundingBoxControls(self, marker):
        container_control = InteractiveMarkerControl()
        container_control.always_visible = True
        container_control.markers = [marker]
        move_x_control = InteractiveMarkerControl()
        move_x_control.name = "move_x"
        move_x_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        move_x_control.always_visible = True
        move_x_control.orientation.w = marker.pose.orientation.w
        move_x_control.orientation.x = marker.pose.orientation.x + marker.scale.x
        # move_x_control.orientation.y = marker.pose.orientation.y
        # move_x_control.orientation.z = marker.pose.orientation.z
        return [container_control,move_x_control]
