from pickle import NONE, TRUE
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QActionGroup, QAction, QToolBar
from PyQt5.QtGui import QIcon
from python_qt_binding.QtCore import Qt
import os
import rospy
import rospkg

# tool bar for rviz
class RvizToolBar(QToolBar):
    # rvizTopic is the topic where pc2 messages are send to display on the rviz main frame
    # pointCloudTopic is the topic where we read messeges from our rosbag
    def __init__(self, rviz_frame):
        super(RvizToolBar, self).__init__()
        self.tool_manager = rviz_frame.getManager().getToolManager()
        self.interatction_tool = self.tool_manager.getTool(0)
        self.selection_tool = self.tool_manager.getTool(1)
        self.icon_path = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource','icons')
        
        self.initUI()

    def initUI(self):
        self.tool_group = QActionGroup(self)
        interaction_tool_icon_path  = os.path.join(self.icon_path,"cursor-move.svg")
        selection_icon_path  = os.path.join(self.icon_path,"cursor-pointer.svg")

        interaction_icon =QIcon(interaction_tool_icon_path)
        interaction_tool_button = QAction(interaction_icon,"Interaction Tool",self)
        interaction_tool_button.setCheckable(True)
        interaction_tool_button.triggered.connect(self._set_interaction_tool)
        selection_icon = QIcon(selection_icon_path)
        select_point_cloud_tool_button = QAction(selection_icon,"Select Point Cloud Tool",self)
        select_point_cloud_tool_button.setCheckable(True)
        select_point_cloud_tool_button.triggered.connect(self._set_selection_tool)
    
        self.interatction_tool.setIcon(interaction_icon)
        self.selection_tool.setIcon(selection_icon)
        self.selection_tool.update(22,2)
        self.addAction(interaction_tool_button)
        self.addAction(select_point_cloud_tool_button)
        self.tool_group.addAction(interaction_tool_button)
        self.tool_group.addAction(select_point_cloud_tool_button)
        self.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        # setting default too to interaction tool
        interaction_tool_button.trigger()
    
    # setting the current tool to interaction tool
    def _set_interaction_tool(self):
        self.tool_manager.setCurrentTool(self.interatction_tool)
    # setting current tool to seleciton tool
    def _set_selection_tool(self):
        self.tool_manager.setCurrentTool(self.selection_tool)
    def _print(self, msg):
        rospy.loginfo("[Rviz Tool Bar] {}".format(msg) )