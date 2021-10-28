import os
import rospy
import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QPushButton, QVBoxLayout, QHBoxLayout,QMainWindow, QLabel, QAction, QMessageBox, QSizePolicy, QFrame
from PyQt5.QtGui import QIcon, QColor
from PyQt5.QtCore import pyqtSlot
import rviz 
from classes import Frame
from classes import AnnotationGroup
import rosbag
from load_rosbag_popup import LoadRosbagPopup
from create_annotation_group_popup import CreateAnnotationGroupPopup
from delete_annotation_group_popup import DeleteAnnotationGroupPopup
from annotation_details_window import AnnotationDetailsWindow
from annotation_list_window import AnnotationListWindow
from .BagPlayer import BagPlayer

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

        # Load in styling for GUI
        style_path = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MaterialDark.qss')
        with open(style_path, 'r') as qss:
            self.style = qss.read()
        self.setStyleSheet(self.style)
        self.initUI()

    def initUI(self):
        # Create RVIZ Visualization Frame
        self.rviz_frame = rviz.VisualizationFrame()
        self.rviz_frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        filePath = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'vizualization_frame.config.rviz')
        reader.readFile( config, filePath)
        self.rviz_frame.load( config )
              
        # Create bag player
        self.bagPlayer = BagPlayer('/rvizdata')

        # Create menubar
        self.create_top_menubar()

        # Load in styling for GUI
        style_path = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MaterialDark.qss')
        with open(style_path, 'r') as qss:
            style = qss.read()
        self.setStyleSheet(style)

        # Set MainWindow's central widget
        # central_widget_layout = QVBoxLayout()
        # central_widget_layout.addWidget(self.rviz_frame)
        # central_widget_layout.addWidget(self.bagPlayer)
        # central_widget = QWidget()
        # central_widget.setLayout(central_widget_layout)
        # self.setCentralWidget(central_widget)


        central_widget_layout = QHBoxLayout()

        rviz_display_layout = QVBoxLayout()
        rviz_display_layout.addWidget(self.rviz_frame)
        rviz_display_layout.addWidget(self.bagPlayer)

        annotations_layout = QVBoxLayout()
        self.annotation_list = AnnotationListWindow(self.annotation_groups)
        self.annotation_details = AnnotationDetailsWindow(self.annotation_groups)
        
        new_annotation_button = QPushButton('Test')
        new_annotation_button.clicked.connect(self.new_annotation)
        annotations_layout.addWidget(new_annotation_button)

        annotations_layout.addWidget(self.annotation_list)
        annotations_layout.addWidget(self.annotation_details)

        central_widget_layout.addLayout(rviz_display_layout, 4)
        central_widget_layout.addLayout(annotations_layout, 1)
        central_widget = QWidget()
        central_widget.setLayout(central_widget_layout)
        self.setCentralWidget(central_widget)

    def create_top_menubar(self):
        # self.statusBar()
        menubar = self.menuBar()
        # File dropdown
        load_rosbag_act = QAction(QIcon('loadfile.png'), 'Load rosbag', self)
        load_rosbag_act.triggered.connect(self.launch_load_rosbag_popup)
        export_rosbag_act = QAction(QIcon('export.png'), 'Export annotations', self)
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

    @pyqtSlot(str, str, name='load_rosbag')
    def get_load_rosbag_data(self, path, topic):
        # If a bag has already been loaded warn the user
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
        self.load_rosbag(path, topic)

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
        self.annotation_list.add_annotation_group(new_annotation_group)

    def launch_delete_annotation_group_popup(self):
        self.dialog = DeleteAnnotationGroupPopup(self.annotation_groups)
        self.dialog.deleted.connect(self.get_delete_annotation_group_data)
        self.dialog.show()

    @pyqtSlot(str, name='delete_annotation_group')
    def get_delete_annotation_group_data(self, group_name):
        group_id = ''
        for group in self.annotation_groups:
            if group.name == group_name:
                group_id = group.id
                self.annotation_groups.remove(group)
                break
        # Update drop down options in annotation details window
        self.annotation_details.delete_annotation_group(group_name)
        # Remove annotation group from annotation list window
        self.annotation_list.delete_annotation_group(group_id)

    # opening a rosbag and loading frames
    def load_rosbag(self, path, topic_name):
        topic_name = str(topic_name)
        path = str(path)
        try:
            self.bag = rosbag.Bag(path)
        except:
            rospy.logerr('Unable to open file: %s', path)
        else:
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
            # Load first frame to viewer here and update our bag player with new frames and loaded bag
            self.bagPlayer.updateBag(topic_name, self.bag, self.frames)

    def new_annotation(self):
        self.annotation_details.prompt_new_annotation()

#import numpy as np

# def inside_test(points , cube3d):
#     """
#     cube3d  =  numpy array of the shape (8,3) with coordinates in the clockwise order. first the bottom plane is considered then the top one.
#     points = array of points with shape (N, 3).

#     Returns the indices of the points array which are outside the cube3d
#     """
#     b1,b2,b3,b4,t1,t2,t3,t4 = cube3d

#     dir1 = (t1-b1)
#     size1 = np.linalg.norm(dir1)
#     dir1 = dir1 / size1

#     dir2 = (b2-b1)
#     size2 = np.linalg.norm(dir2)
#     dir2 = dir2 / size2

#     dir3 = (b4-b1)
#     size3 = np.linalg.norm(dir3)
#     dir3 = dir3 / size3

#     cube3d_center = (b1 + t3)/2.0

#     dir_vec = points - cube3d_center

#     res1 = np.where( (np.absolute(np.dot(dir_vec, dir1)) * 2) > size1 )[0]
#     res2 = np.where( (np.absolute(np.dot(dir_vec, dir2)) * 2) > size2 )[0]
#     res3 = np.where( (np.absolute(np.dot(dir_vec, dir3)) * 2) > size3 )[0]

#     return list( set().union(res1, res2, res3) )