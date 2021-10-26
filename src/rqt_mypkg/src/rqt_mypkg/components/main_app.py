import os
import rospy
import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout,QMainWindow, QLabel, QAction, QMessageBox, QSizePolicy, QFrame
from PyQt5.QtGui import QIcon, QColor
from PyQt5.QtCore import pyqtSlot
import rviz 
from classes import Frame
from classes import AnnotationGroup
import rosbag
from load_rosbag_popup import LoadRosbagPopup
from create_annotation_group_popup import CreateAnnotationGroupPopup
from delete_annotation_group_popup import DeleteAnnotationGroupPopup
from annotation_details import AnnotationDetails
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
        central_widget_layout.setContentsMargins(0, 0, 0, 0)

        rviz_display_layout = QVBoxLayout()
        rviz_display_layout.addWidget(self.rviz_frame)
        rviz_display_layout.addWidget(self.bagPlayer)

        annotations_layout = QVBoxLayout()
        # annotations_layout.setContentsMargins(0, 0, 0, 0)
        self.annotation_details = AnnotationDetails(self.annotation_groups)
        test2 = QWidget()
        test2.setProperty('class', 'Test')

        title = QLabel('Annotation Details')
        title.setProperty('class', 'widgetTitle')
        annotations_layout.addWidget(title)
        annotations_layout.addWidget(self.annotation_details)
        annotations_layout.addWidget(test2)

        central_widget_layout.addLayout(rviz_display_layout, 3)
        central_widget_layout.addLayout(annotations_layout, 1)
        central_widget = QWidget()
        central_widget.setLayout(central_widget_layout)
        # central_widget.setStyleSheet(self.style)
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
        self.annotation_details.add_annotation_group(new_annotation_group)

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
