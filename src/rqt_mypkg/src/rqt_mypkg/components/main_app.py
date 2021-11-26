import os
from sys import modules
import uuid
from interactive_markers.interactive_marker_server import MarkerContext
import rospy
import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout,QMainWindow, QAction, QMessageBox, QAbstractButton
from PyQt5.QtGui import QIcon, QColor, QFontDatabase, QIcon
from PyQt5.QtCore import pyqtSlot
import rviz 
from std_msgs.msg import ColorRGBA
from classes import Frame, Annotator, AnnotationGroup
import rosbag
from load_rosbag_popup import LoadRosbagPopup
from export_rosbag_popup import ExportRosbagPopup
from create_annotation_group_popup import CreateAnnotationGroupPopup
from delete_annotation_group_popup import DeleteAnnotationGroupPopup
from annotation_details_window import AnnotationDetailsWindow
from annotation_list_window import AnnotationListWindow
from .BagPlayer import BagPlayer
from annotation_msgs.msg import annotation_group
from .auxiliary_functions import get_annotation_group_by_id, get_valid_ColorRGBA_MSG, get_valid_QColor, msg_from_frame, frame_from_msg
from .rviz_toolbar import RvizToolBar
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

        self.resource_path = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource')

        self.initUI()

    def initUI(self):
        # Change default font
        QFontDatabase.addApplicationFont(os.path.join(self.resource_path, 'fonts', 'AtkinsonHyperlegible-Regular.ttf'))

        # Load in styling for GUI
        with open(os.path.join(self.resource_path, 'dark.qss'), 'r') as qss:
            self.style = qss.read()
        self.setStyleSheet(self.style)
        
        # Create RVIZ Visualization Frame
        self.rviz_frame = self._create_rviz_frame()
        # Tool menu bar
        self.rviz_tool_bar = RvizToolBar(self.rviz_frame)

        # Create bag player
        self.bagPlayer = BagPlayer('/rvizdata')
        
        # Annotator
        self.annotator = None
        # Create menubar
        self.create_top_menubar()

    
        rviz_display_layout = QVBoxLayout()
        rviz_display_layout.addWidget(self.rviz_tool_bar)
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

    def create_top_menubar(self):
        # self.statusBar()
        menubar = self.menuBar()
        # File dropdown
        icon_path = os.path.join(self.resource_path, 'icons')
        load_rosbag_act = QAction(QIcon(os.path.join(icon_path, 'tray-arrow-down.svg')), 'Load rosbag', self)
        load_rosbag_act.triggered.connect(self.launch_load_rosbag_popup)
        export_rosbag_act = QAction(QIcon(os.path.join(icon_path, 'tray-arrow-up.svg')), 'Export annotations', self)
        export_rosbag_act.triggered.connect(self.launch_export_rosbag_popup)
        file_menu = menubar.addMenu('File')
        file_menu.addAction(load_rosbag_act)
        file_menu.addAction(export_rosbag_act)
        # Annotation group dropdown
        create_annotation_group_act = QAction(QIcon(os.path.join(icon_path, 'plus-thick.svg')), 'Create annotation group', self)
        create_annotation_group_act.triggered.connect(self.launch_create_annotation_group_popup)
        delete_annotation_group_act = QAction(QIcon(os.path.join(icon_path, 'delete.svg')), 'Delete annotation group', self)
        delete_annotation_group_act.triggered.connect(self.launch_delete_annotation_group_popup)
        annotation_group_menu = menubar.addMenu('Annotation group')
        annotation_group_menu.addAction(create_annotation_group_act)
        annotation_group_menu.addAction(delete_annotation_group_act)        

    def launch_create_annotation_group_popup(self):
        self.creat_annotation_group_popup = CreateAnnotationGroupPopup(self.annotation_groups)
        self.creat_annotation_group_popup.created.connect(self.create_annotation_group)
        self.creat_annotation_group_popup.show()

    @pyqtSlot(str, QColor, name='create_annotation_group')
    def create_annotation_group(self, group_name, color):
        new_annotation_group = AnnotationGroup(group_name, color)
        self.annotation_groups.append(new_annotation_group)
        # Update drop down options in annotation details window
        self.annotation_details.add_annotation_group(group_name)
        # Add annotation group to annotation list window
        self.annotation_list.refresh(self.annotator.currentFrame)

    def launch_delete_annotation_group_popup(self):
        self.delete_annotation_group_popup = DeleteAnnotationGroupPopup(self.annotation_groups)
        self.delete_annotation_group_popup.deleted.connect(self.delete_annotation_group)
        self.delete_annotation_group_popup.show()

    @pyqtSlot(str, name='delete_annotation_group')
    def delete_annotation_group(self, group_name):
        for group in self.annotation_groups:
            if group.name == group_name:
                self.annotation_groups.remove(group)
                break
        # Update drop down options in annotation details window
        self.annotation_details.delete_annotation_group(group_name)
        # Remove annotation group from annotation list window
        self.annotation_list.refresh(self.annotator.currentFrame)

    def launch_load_rosbag_popup(self): 
        self.load_rosbag_popup = LoadRosbagPopup()
        self.load_rosbag_popup.submitted.connect(self.load_rosbag)
        self.load_rosbag_popup.show()

    # opening a rosbag and loading frames
    @pyqtSlot(str, str, str, str, name='load_rosbag')
    def load_rosbag(self, _path, _lidar_topic_name, _annot_topic_name, _group_topic_name):
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
        
        path, self.input_path = str(_path), str(_path)
        lidar_topic_name, self.input_lidar_topic_name = str(_lidar_topic_name), str(_lidar_topic_name)
        annot_topic_name = str(_annot_topic_name)
        group_topic_name = str(_group_topic_name)
        try:
            self.bag = rosbag.Bag(path)
        except:
            rospy.logerr('Unable to open file: %s', path)
        else:
            # Ensure topic names are in the bag and get the total number of messages for the progress bar
            total_msgs = 0
            annot_topic_exists = False
            group_topic_exists = False
            if lidar_topic_name != '':
                if lidar_topic_name not in self.bag.get_type_and_topic_info()[1].keys():
                    rospy.logerr('Topic \'%s\' not found in rosbag', lidar_topic_name)
                    self.bag.close()
                    return
                lidar_topic_exists = True
                total_msgs += self.bag.get_message_count(lidar_topic_name)
            else:
                return
            if annot_topic_name != '':
                if annot_topic_name not in self.bag.get_type_and_topic_info()[1].keys():
                    rospy.logerr('Topic \'%s\' not found in rosbag', annot_topic_name)
                else:
                    annot_topic_exists = True
                    total_msgs += self.bag.get_message_count(annot_topic_name)
            if group_topic_name != '':
                if group_topic_name not in self.bag.get_type_and_topic_info()[1].keys():
                    rospy.logerr('Topic \'%s\' not found in rosbag', group_topic_name)
                else:
                    group_topic_exists = True
                    total_msgs += self.bag.get_message_count(group_topic_name)
            
            self.load_rosbag_popup.set_progress_bar_range(total_msgs)

            if lidar_topic_exists:
                # Create empty list of Frames for each message
                self.frames = [Frame]*self.bag.get_message_count(lidar_topic_name)
                # Iterate through msgs and populate the empty list
                index = 0
                for topic, msg, t in self.bag.read_messages(topics=[lidar_topic_name]):
                    frame = Frame(t)
                    self.frames[index] = frame
                    index += 1
                    self.load_rosbag_popup.increment_progress_bar()
            if annot_topic_exists:
                if len(self.frames) == 0:
                    self.frames = [Frame]*self.bag.get_message_count(annot_topic_name)
                index = 0
                for topic, msg, t in self.bag.read_messages(topics=[annot_topic_name]):
                    frame = frame_from_msg(t, msg)
                    self.frames[index].annotations = frame
                    index += 1
                    self.load_rosbag_popup.increment_progress_bar()
            
            # Create Annotator with list of frames
            self.annotator  = Annotator(self.frames)
            # Set annotation list window's set of frames
            self.annotation_list.set_frames(self.frames)

            if group_topic_exists:
                for group in self.annotation_groups:
                    self.delete_annotation_group(group.name)
                for topic, msg, t in self.bag.read_messages(topics=[group_topic_name]):
                    group_color = get_valid_QColor(msg.color)
                    group = AnnotationGroup(msg.name, group_color, msg.id)
                    if group not in self.annotation_groups:
                        self.annotation_groups.append(group)
                        # Update drop down options in annotation details window
                        self.annotation_details.add_annotation_group(msg.name)
                        # Remove annotation group from annotation list window
                        self.annotation_list.refresh(self.annotator.currentFrame)
                        self.load_rosbag_popup.increment_progress_bar()

            # Load first frame to viewer here and update our bag player with new frames and loaded bag
            self.bagPlayer.updateBag(lidar_topic_name, self.bag, self.frames,self.annotator)
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

    def launch_export_rosbag_popup(self):
        # If a bag has not been loaded notify the user and abort
        if len(self.frames) == 0:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText('A rosbag has not been loaded.')
            msg.setInformativeText('You cannot export an empty rosbag.')
            msg.setWindowTitle('Error')
            # msg.setDetailedText('The details are as follows:')
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()
            return
        self.export_rosbag_popup = ExportRosbagPopup()
        self.export_rosbag_popup.submitted.connect(self.export_rosbag)
        self.export_rosbag_popup.show()

    @pyqtSlot(str, str, bool, name='export_rosbag') 
    def export_rosbag(self, path, topic_name, export_lidar):
        topic_name = str(topic_name)
        path = str(path)
        if self.input_path == path:
	        rospy.logerr("Error: Cannot export to the input rosbag")
        with rosbag.Bag(path, 'w') as outbag:
            total_msgs = len(self.frames) + len(self.annotation_groups)
            self.export_rosbag_popup.set_progress_bar_range(total_msgs)
            for f in self.frames:
                #load this frame from input rosbag
                for topic, msg, t in self.bag.read_messages(topics=[self.input_lidar_topic_name], start_time=f.timestamp, end_time=f.timestamp):
                    outbag.write(topic, msg, t)
                msg = msg_from_frame(f)
                outbag.write(topic_name, msg, f.timestamp)
                self.export_rosbag_popup.increment_progress_bar()
            for g in self.annotation_groups:
                group = annotation_group()
                group.color = get_valid_ColorRGBA_MSG(g.color)
                group.name = g.name
                group.id = g.id
                outbag.write('/group', group)
                self.export_rosbag_popup.increment_progress_bar()
    
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

    # creating rviz frame main window
    def _create_rviz_frame(self):
        rviz_frame = rviz.VisualizationFrame()
        rviz_frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        rviz_config_file_path = os.path.join(self.resource_path, 'vizualization_frame.config.rviz')
        reader.readFile( config, rviz_config_file_path)
        rviz_frame.load( config )
        for button in rviz_frame.findChildren(QAbstractButton):
            button.hide()
        # this removes the RVIZ frame menu bar
        rviz_frame.setMenuBar( None )
        # removing status bar from RVIZ frame
        rviz_frame.setStatusBar( None )
        return rviz_frame