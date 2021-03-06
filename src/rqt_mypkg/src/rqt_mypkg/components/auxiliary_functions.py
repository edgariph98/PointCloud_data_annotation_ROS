from std_msgs.msg import ColorRGBA
from PyQt5.QtGui import QColor
from annotation_msgs.msg import frame, annotation
from .classes import Frame, Annotator, Annotation, AnnotationGroup
import os
def deleteItemsOfLayout(layout):
     if layout is not None:
         while layout.count():
             item = layout.takeAt(0)
             widget = item.widget()
             if widget is not None:
                 widget.setParent(None)
             else:
                 deleteItemsOfLayout(item.layout())

def get_annotation_group_by_id(annotation_groups, group_id):
    # Returns the annotation_group with the matching id or None if no match is found
    return next((elem for elem in annotation_groups if elem.id == group_id), None)

def get_annotation_group_by_name(annotation_groups, name):
    # Returns the annotation_group with the matching name or None if no match is found
    return next((elem for elem in annotation_groups if elem.name == name), None)

# gets a valid color Rgba msg from a QColor
def get_valid_ColorRGBA_MSG(q_color):
    rgbaValues = q_color.getRgb()
    valid_color = ColorRGBA()
    valid_color.r = float(rgbaValues[0]) / 255.0
    valid_color.g = float(rgbaValues[1]) / 255.0
    valid_color.b = float(rgbaValues[2]) / 255.0
    valid_color.a = 175 / 255.0
    return valid_color

def get_valid_QColor(color_rgba):
    r = int(color_rgba.r * 255.0)
    g = int(color_rgba.g * 255.0)
    b = int(color_rgba.b * 255.0)
    valid_q_color = QColor.fromRgb(r, g, b)
    return valid_q_color

def msg_from_frame(_frame):
    msg = frame()
    msg.id = str(_frame.id)
    for a in _frame.annotations:
        annot = annotation()
        annot.id = a.id
        annot.label = a.label
        annot.group = a.group_id
        annot.marker = a.marker
        annot.captured_point_cloud = a.captured_point_cloud
        msg.annotations.append(annot)
    return msg

def frame_from_msg(t, _msg):
    frame = []
    for a in _msg.annotations:
        annot = Annotation(a.id, a.label, a.group, a.marker, a.marker.color, a.captured_point_cloud)
        frame.append(annot)
    return frame

def get_ros_version():
    """
    returns ros distribution version on the string
    """
    return os.environ.get("ROS_DISTRO")