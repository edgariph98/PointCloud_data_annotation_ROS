import os
from sys import modules
import uuid
from interactive_markers.interactive_marker_server import MarkerContext
import rospy
import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QPushButton, QVBoxLayout, QHBoxLayout,QMainWindow, QLabel, QAction, QMessageBox, QLineEdit
from PyQt5.QtGui import QIcon, QColor, QFontDatabase, QFont
from PyQt5.QtCore import pyqtSlot
import rviz 
from std_msgs.msg import ColorRGBA
from classes import Frame, Annotator, Annotation, AnnotationGroup
import rosbag
from load_rosbag_popup import LoadRosbagPopup
from export_rosbag_popup import ExportRosbagPopup
from create_annotation_group_popup import CreateAnnotationGroupPopup
from delete_annotation_group_popup import DeleteAnnotationGroupPopup
from annotation_details_window import AnnotationDetailsWindow
from annotation_list_window import AnnotationListWindow
from .BagPlayer import BagPlayer
from annotation_msgs.msg import frame, annotation, annotation_group
from .auxiliary_functions import get_annotation_group_by_id, get_valid_ColorRGBA_MSG, get_valid_QColor

# Main App widget to be imported in RQT Plugin
class MainApp(QMainWindow):
    def __init__(self):
        super(MainApp, self).__init__()
        # List of rosbag frames
        self.frames = []
        # List of annotation groups
        self.annotation_groups = []
        # rosbag object
        self.bag = None

        # Change default font
        font_path = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'fonts', 'AtkinsonHyperlegible-Regular.ttf')
        QFontDatabase.addApplicationFont(font_path)

        # Load in styling for GUI
        style_path = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'dark.qss')
        with open(style_path, 'r') as qss:
            self.style = qss.read()
        self.setStyleSheet(self.style)
        
        self.initUI()
        self.input_path = ''
        self.input_topic = ''

    def initUI(self):
        # Create RVIZ Visualization Frame
        self.rviz_frame = rviz.VisualizationFrame()
        self.rviz_frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        filePath = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'vizualization_frame.config.rviz')
        reader.readFile( config, filePath)
        self.rviz_frame.load( config )
        # this removes the RVIZ fram menu bar
        self.rviz_frame.setMenuBar( None )
              
        # Create bag player
        self.bagPlayer = BagPlayer('/rvizdata')

        # Annotator
        self.annotator = None
        # Create menubar
        self.create_top_menubar()

        rviz_display_layout = QVBoxLayout()
        rviz_display_layout.addWidget(self.rviz_frame)
        rviz_display_layout.addWidget(self.bagPlayer)

        # Create annotation details and list windows and add them to vertical layout
        annotations_layout = QVBoxLayout()
        self.annotation_list = AnnotationListWindow(self.frames, self.annotation_groups)
        self.annotation_details = AnnotationDetailsWindow(self.annotation_groups)
        annotations_layout.addWidget(self.annotation_list)
        annotations_layout.addWidget(self.annotation_details)

        # Set MainWindow's central widget
        central_widget_layout = QHBoxLayout()
        central_widget_layout.addLayout(rviz_display_layout, 4)
        central_widget_layout.addLayout(annotations_layout, 1)
        central_widget = QWidget()
        central_widget.setLayout(central_widget_layout)
        self.setCentralWidget(central_widget)

        # self.load_rosbag('/home/trevor/Downloads/mcity1.bag', '/lidar_left/velodyne_points')

    def create_top_menubar(self):
        # self.statusBar()
        menubar = self.menuBar()
        # File dropdown
        load_rosbag_act = QAction(QIcon('loadfile.png'), 'Load rosbag', self)
        load_rosbag_act.triggered.connect(self.launch_load_rosbag_popup)
        export_rosbag_act = QAction(QIcon('export.png'), 'Export annotations', self)
        export_rosbag_act.triggered.connect(self.launch_export_rosbag_popup)
        file_menu = menubar.addMenu('File')
        file_menu.addAction(load_rosbag_act)
        file_menu.addAction(export_rosbag_act)
        # Annotation group dropdown
        create_annotation_group_act = QAction(QIcon('new.png'), 'Create annotation group', self)
        create_annotation_group_act.triggered.connect(self.launch_create_annotation_group_popup)
        delete_annotation_group_act = QAction(QIcon('delete.png'), 'Delete annotation group', self)
        delete_annotation_group_act.triggered.connect(self.launch_delete_annotation_group_popup)
        annotation_group_menu = menubar.addMenu('Annotation group')
        annotation_group_menu.addAction(create_annotation_group_act)
        annotation_group_menu.addAction(delete_annotation_group_act)        

    def launch_load_rosbag_popup(self): 
        self.dialog = LoadRosbagPopup()
        self.dialog.submitted.connect(self.get_load_rosbag_data)
        self.dialog.show()
        
    def launch_export_rosbag_popup(self):
        self.dialog = ExportRosbagPopup()
        self.dialog.submitted.connect(self.get_export_rosbag_data)
        self.dialog.show()

    @pyqtSlot(str, str, str, str, name='load_rosbag')
    def get_load_rosbag_data(self, path, topic, annot, group):
        # If a bag has already been loaded, warn the user
        if len(self.frames) != 0:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText('A rosbag has already been loaded.')
            msg.setInformativeText('Opening another rosbag will cause any unsaved annotations to be permanently deleted.')
            msg.setWindowTitle('Warning')
            # msg.setDetailedText('The details are as follows:')
            msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
            retval = msg.exec_()
            if retval == QMessageBox.Cancel:
                return
        self.load_rosbag(path, topic, annot, group)
      
    @pyqtSlot(str, str, bool, name='export_rosbag')    
    def get_export_rosbag_data(self, path, topic, lidar):
    	# If a bag has not already been loaded warn the user
        if len(self.frames) == 0:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText('A rosbag has not already been loaded.')
            msg.setInformativeText('Exporting without any frames or annotations will not create a rosbag.')
            msg.setWindowTitle('Warning')
            # msg.setDetailedText('The details are as follows:')
            msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
            retval = msg.exec_()
            if retval == QMessageBox.Cancel:
                return
    	self.export_rosbag(path, topic, lidar)

    def launch_create_annotation_group_popup(self):
        self.dialog = CreateAnnotationGroupPopup(self.annotation_groups)
        self.dialog.created.connect(self.get_create_annotation_group_data)
        self.dialog.show()

    @pyqtSlot(str, QColor, name='create_annotation_group')
    def get_create_annotation_group_data(self, group_name, color):
        new_annotation_group = AnnotationGroup(group_name, color)
        self.annotation_groups.append(new_annotation_group)
        # Update drop down options in annotation details window
        self.annotation_details.add_annotation_group(group_name)
        # Add annotation group to annotation list window
        self.annotation_list.refresh(self.annotator.currentFrame)

    def launch_delete_annotation_group_popup(self):
        self.dialog = DeleteAnnotationGroupPopup(self.annotation_groups)
        self.dialog.deleted.connect(self.get_delete_annotation_group_data)
        self.dialog.show()

    @pyqtSlot(str, name='delete_annotation_group')
    def get_delete_annotation_group_data(self, group_name):
        for group in self.annotation_groups:
            if group.name == group_name:
                self.annotation_groups.remove(group)
                break
        # Update drop down options in annotation details window
        self.annotation_details.delete_annotation_group(group_name)
        # Remove annotation group from annotation list window
        self.annotation_list.refresh(self.annotator.currentFrame)

    # opening a rosbag and loading frames
    def load_rosbag(self, path, topic_name, annot_topic, group_topic):
        topic_name = str(topic_name)
        path = str(path)
        try:
            self.bag = rosbag.Bag(path)
        except:
            rospy.logerr('Unable to open file: %s', path)
        else:
            self.input_path = path
            self.input_topic = topic_name
            if topic_name != "":
                if topic_name not in self.bag.get_type_and_topic_info()[1].keys():
                    rospy.logerr('Topic \'%s\' not found in rosbag', topic_name)
                    self.bag.close()
                    return
                # Create empty list of Frames for each message
                self.frames = [Frame]*self.bag.get_message_count(topic_name)
                # Iterate through msgs and populate the empty list
                index = 0
                for topic, msg, t in self.bag.read_messages(topics=[topic_name]):
                    frame = Frame(t)
                    self.frames[index] = frame
                    index += 1
            if annot_topic != "":
                if annot_topic not in self.bag.get_type_and_topic_info()[1].keys():
                    rospy.logerr('Topic \'%s\' not found in rosbag', annot_topic)
                    self.bag.close()
                    return
                if len(self.frames) == 0:
                    self.frames = [Frame]*self.bag.get_message_count(annot_topic)
                index  = 0
                for topic, msg, t in self.bag.read_messages(topics=[annot_topic]):
                    frame = self.frame_from_msg(t, msg)
                    self.frames[index].annotations = frame
                    index += 1
            
            # Create Annotator with list of frames
            self.annotator  = Annotator(self.frames)
            # Set annotation list window's set of frames
            self.annotation_list.set_frames(self.frames)

            if group_topic != "":
                if group_topic not in self.bag.get_type_and_topic_info()[1].keys():
                    rospy.logerr('Topic \'%s\' not found in rosbag', group_topic)
                    self.bag.close()
                for topic, msg, t in self.bag.read_messages(topics=[group_topic]):
                    group_color = get_valid_QColor(msg.color)
                    group = AnnotationGroup(msg.name, group_color, msg.id)
                    if group not in self.annotation_groups:
                        self.annotation_groups.append(group)
                        # Update drop down options in annotation details window
                        self.annotation_details.add_annotation_group(msg.name)
                        # Remove annotation group from annotation list window
                        self.annotation_list.refresh(self.annotator.currentFrame)
            # # Create Annotator with list of frames and annotation groups
            # self.annotator  = Annotator(self.frames)    

            # # Load first frame to viewer here and update our bag player with new frames and loaded bag
            # self.bagPlayer.updateBag(topic_name, self.bag, self.frames,self.annotator)
            # # Connect signals and slots between Annotator and annotation_details_window
            # if topic_name not in self.bag.get_type_and_topic_info()[1].keys():
            #     rospy.logerr('Topic \'%s\' not found in rosbag', topic_name)
            #     self.bag.close()
            #     return
            # # Create empty list of Frames for each message
            # self.frames = [Frame]*self.bag.get_message_count(topic_name)
            # # Iterate through msgs and populate the empty list
            # index = 0
            # for topic, msg, t in self.bag.read_messages(topics=[topic_name]):
            #     frame = Frame(t)
            #     self.frames[index] = frame
            #     index += 1
            # Load first frame to viewer here and update our bag player with new frames and loaded bag
            self.bagPlayer.updateBag(topic_name, self.bag, self.frames,self.annotator)
            # Connect signals and slots between components
            self.annotator.pending_annotation_marker.connect(self.annotation_details.get_pending_annotation_marker)
            self.annotator.rviz_cancelled_new_annotation.connect(self.annotation_details.rviz_cancelled_new_annotation)
            self.annotator.annotation_details.connect(self.annotation_details.get_annotation_details)
            self.annotator.delete_annotation_signal.connect(self.annotation_list.delete_annotation)
            self.annotator.delete_annotation_signal.connect(self.annotation_details.delete_annotation)

            self.annotation_details.confirmed_annotation.connect(self.get_confirmed_annotation)
            self.annotation_details.cancelled_new_annotation.connect(self.cancelled_new_annotation)
            self.annotation_details.confirmed_delete_annotation.connect(self.annotator.delete_annotation)
            self.annotation_details.confirmed_delete_annotation.connect(self.annotation_list.delete_annotation)

            self.bagPlayer.changed_frame.connect(self.annotation_list.refresh)
            self.bagPlayer.changed_frame.connect(self.annotation_details.clear_fields)
            self.annotation_list.annotation_details.connect(self.annotation_details.get_annotation_details)

    @pyqtSlot(str, str, str, name='confirm_annotation')
    def get_confirmed_annotation(self, label, annotation_id, group_id):
        group = get_annotation_group_by_id(self.annotation_groups, group_id)
        print("Recieved from annotation_details_window id :{}, Label: {}, Group id: {}, Group Name: {}".format(
            annotation_id, label, group_id, group.name))
        valid_color = get_valid_ColorRGBA_MSG(group.color)
        self.annotator.createAnnotation(group_id,label,annotation_id,valid_color)
        self.annotation_list.refresh(self.annotator.currentFrame)

    @pyqtSlot(name='cancelled_new_annotation')
    def cancelled_new_annotation(self):
        print("Annotation just got cancelled")
        self.annotator.remove_selection()

    # Todo
    # 1. Emit signal for new annotation point info 'self.pending_annotation_points.emit(values whatever they are)'

    def createAnnotation(self,label,groupName,r,g,b):
        color = ColorRGBA()
        color.r = float(r)
        color.g = float(g)
        color.b = float(b)
        color.a = 0.3
        self.annotator.createAnnotation(groupName,label,str(uuid.uuid4()), color)

    def export_rosbag(self, path, topic_name, lidar):
        topic_name = str(topic_name)
        path = str(path)
        if self.input_path == path:
	        rospy.logerr("Error: Cannot export to the input rosbag")
        with rosbag.Bag(path, 'w') as outbag:
            for f in self.frames:
                #load this frame from input rosbag
                for topic, msg, t in self.bag.read_messages(topics=[self.input_topic], start_time=f.timestamp, end_time=f.timestamp):
                    outbag.write(topic, msg, t)
                msg = self.msg_from_frame(f)
                outbag.write(topic_name, msg, f.timestamp)
            for g in self.annotation_groups:
                group = annotation_group()
                group.color = get_valid_ColorRGBA_MSG(g.color)
                group.name = g.name
                group.id = g.id
                outbag.write('/group', group)
        		
    def msg_from_frame(self, _frame):
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
    
    def frame_from_msg(self, t, _msg):
        frame = []
        for a in _msg.annotations:
            rospy.loginfo("annotation found")
            annot = Annotation(a.id, a.label, a.group, a.marker, a.marker.color, a.captured_point_cloud)
            frame.append(annot)
        return frame

