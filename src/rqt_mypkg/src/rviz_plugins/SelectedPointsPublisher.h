////////////////////////////////////////////////////////////////////////////////
//
// Filename:      selected_points_topic.h
// Last change:   2013-11-21
// Authors:       Bartels, Philipp (mail@pBartels.net)
// Documentation: http://docs.ros.org/api/rviz/html/
// Version:       1.0.0
//
//////////////////////////////// DOCUMENTATION /////////////////////////////////
//
// Fork of the rviz::SelectionTool:
// Drag with the left button to select objects in the 3D scene.
// Hold the Alt key to change viewpoint as in the Move tool.
// Additionally publishes selected points on /selected_points topic.
//
/////////////////////////////////// LICENSE ////////////////////////////////////
//
// Copyright (C) 2013 Robotics & Biology Laboratory (RBO) TU Berlin
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////// CHANGELOG ///////////////////////////////////
//
// Version 1.0.0 (2013-11-21)
//
//////////////////////////////////// NOTES /////////////////////////////////////
//
// TODO:
//
////////////////////////////////////////////////////////////////////////////////

#ifndef SELECTED_POINTS_PUBLISHER_H
#define SELECTED_POINTS_PUBLISHER_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <ros/node_handle.h>
# include <ros/publisher.h>

# include "rviz/tool.h"

# include <QCursor>
# include <QObject>
#endif

#include "rviz/default_plugin/tools/selection_tool.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include "rviz/selection/forwards.h"
#include <pcl/filters/extract_indices.h>

namespace rqt_mypkg
{

class SelectedPointsPublisher;

class SelectedPointsPublisher : public rviz::SelectionTool
{
Q_OBJECT
public:
  SelectedPointsPublisher();
  virtual ~SelectedPointsPublisher();

  /*
   * Hooks on rviz::SelectionTool::processMouseEvent() to get and publish
   * selected points
   */
  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

  virtual int processKeyEvent( QKeyEvent* event, rviz::RenderPanel* panel );

public Q_SLOTS:
  /*
   * Loads publishers an subscribers
   */
  void updateTopic();

  void annotation_completed_callback(const visualization_msgs::MarkerConstPtr &boundingBoxMarker);

protected:
  // grabs the selected points and generates bounding boX marker
  int _processSelectedAreaAndFindPoints();
  // resets selection and removes bounding box marker displaying
  void removeSelectedPoints();
  // removes current bounding box marker
  void remove_selected_bounding_box_marker();
  ros::NodeHandle nh_;
  // publisher for publushing bounding box marker of selection made
  ros::Publisher bb_marker_pub_;
  // subscriber used to update selection once annotation has been created and completed
  ros::Subscriber annotation_completed_subscriber;
  // publisher for the annotation selected 
  ros::Publisher annotation_selected_created_publisher;
  // publisher to when an annotation has been cleared
  ros::Publisher annotation_selected_removed_publisher;
  
  std::string tf_frame_;
  // strings for subscribers and puublishers
  std::string bb_marker_topic_;
  std::string annotation_completed_topic;
  std::string annotation_selected_created_topic;
  std::string annotation_selected_removed_topic;

  bool selecting_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pc_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr selected_segment_pc_;

  pcl::ExtractIndices<pcl::PointXYZRGB>::Ptr extract_indices_filter_;

  // selected points
  int num_selected_points_;
};
} // end namespace rviz_plugin_selected_points_publisher

#endif // SELECTED_POINTS_PUBLISHER_H

////////////////////////////////////////////////////////////////////////////////