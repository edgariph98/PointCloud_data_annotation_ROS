#include "rviz/selection/selection_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/view_manager.h"
#include "rviz/view_controller.h"
#include "rviz/tool_manager.h"

#include "OGRE/OgreCamera.h"


#include "SelectedPointsPublisher.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <QVariant>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>

#include <pcl/filters/impl/box_clipper3D.hpp>


#include <visualization_msgs/Marker.h>

#include <pcl/filters/crop_box.h>
#include <pcl/common/angles.h>
#include <annotation_msgs/Annotation.h>
#include <std_msgs/Bool.h>
namespace rqt_mypkg
{
SelectedPointsPublisher::SelectedPointsPublisher()
{
    updateTopic();
}

SelectedPointsPublisher::~SelectedPointsPublisher()
{
}

void SelectedPointsPublisher::updateTopic()
{
    nh_.param("frame_id", tf_frame_, std::string("/base_link"));

    // strings for topics
    bb_marker_topic_                    = std::string("/selection/bounding_box_marker");
    annotation_completed_topic          = std::string("/selection/annotation_completed");
    annotation_selected_created_topic   = std::string("/selection/annotation_selected_created");
    annotation_selected_removed_topic   = std::string("/selection/annotation_selected_removed");
    

    // subscriber to update and remove selection, once annotation has been created
    annotation_completed_subscriber = nh_.subscribe(annotation_completed_topic.c_str(),1,&SelectedPointsPublisher::annotation_completed_callback,this);
    // publisher to publish bounding box marker that captures the points selected
    bb_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(bb_marker_topic_.c_str(), 1);
    // publisher to publish annotation msg when an annotation has been selected
    annotation_selected_created_publisher   = nh_.advertise<annotation_msgs::Annotation>(annotation_selected_created_topic.c_str(),1);
    // publisher to send a boolean message when the current selection has been cleared
    annotation_selected_removed_publisher   = nh_.advertise<std_msgs::Bool>(annotation_selected_removed_topic.c_str(),1);
    ROS_INFO_STREAM_NAMED("SelectedPointsPublisher.updateTopic", "Publishing selected Annotation msg, with Bounding Box Marker, and the PointCloud2 Data on topic: " <<  nh_.resolveName (annotation_selected_created_topic) );//<< " with frame_id " << context_->getFixedFrame().toStdString() );
    ROS_INFO_STREAM_NAMED("SelectedPointsPublisher.updateTopic", "Publishing selected bounding box marker on topic:                                                " <<  nh_.resolveName (bb_marker_topic_) );//<< " with frame_id " << context_->getFixedFrame().toStdString() );

    current_pc_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    num_selected_points_ = 0;
}

// annotation confirmations has been created, we clear current selection
void SelectedPointsPublisher::annotation_completed_callback(const visualization_msgs::MarkerConstPtr &boundingBoxMarker){
    this->removeSelectedPoints();
}

int SelectedPointsPublisher::processKeyEvent( QKeyEvent* event, rviz::RenderPanel* panel )
{
        if(event->type() == QKeyEvent::KeyPress)
        {
            if(event->key() == 'c' || event->key() == 'C')
            {
                ROS_INFO_STREAM_NAMED( "SelectedPointsPublisher::processKeyEvent", "Cleaning ALL previous selection (selected area and points).");
                this->removeSelectedPoints();
            }
        }
}

int SelectedPointsPublisher::processMouseEvent( rviz::ViewportMouseEvent& event )
{
    int flags = rviz::SelectionTool::processMouseEvent( event );

    // determine current selection mode if alt, no selection
    if( event.alt() )
    {
        selecting_ = false;
    }
    else
    {
        if( event.leftDown())
        {   
            if(event.shift() || event.control()){
                this->remove_selected_bounding_box_marker();
            }
            selecting_ = true;
        }
    }

    if( selecting_ )
    {
        if( event.leftUp() )
        {
            ROS_INFO_STREAM_NAMED( "SelectedPointsPublisher.processKeyEvent", "Using selected area to find a new bounding box and publish the points inside of it");
            this->_processSelectedAreaAndFindPoints();
        }
    }
    return flags;
}

int SelectedPointsPublisher::_processSelectedAreaAndFindPoints()
{
    rviz::SelectionManager* sel_manager = context_->getSelectionManager();
    rviz::M_Picked selection = sel_manager->getSelection();
    rviz::PropertyTreeModel *model = sel_manager->getPropertyModel();
    int num_points = model->rowCount();
    ROS_INFO_STREAM_NAMED( "SelectedPointsPublisher._processSelectedAreaAndFindPoints", "Number of points in the selected area: " << num_points);

    // Generate a ros point cloud message with the selected points in rviz
    sensor_msgs::PointCloud2 selected_points_ros;
    selected_points_ros.header.frame_id = context_->getFixedFrame().toStdString();
    selected_points_ros.height = 1;
    selected_points_ros.width = num_points;
    selected_points_ros.point_step = 3 * 4;
    selected_points_ros.row_step = num_points * selected_points_ros.point_step;
    selected_points_ros.is_dense = false;
    selected_points_ros.is_bigendian = false;

    selected_points_ros.data.resize( selected_points_ros.row_step );
    selected_points_ros.fields.resize( 3 );

    selected_points_ros.fields[0].name = "x";
    selected_points_ros.fields[0].offset = 0;
    selected_points_ros.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    selected_points_ros.fields[0].count = 1;

    selected_points_ros.fields[1].name = "y";
    selected_points_ros.fields[1].offset = 4;
    selected_points_ros.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    selected_points_ros.fields[1].count = 1;

    selected_points_ros.fields[2].name = "z";
    selected_points_ros.fields[2].offset = 8;
    selected_points_ros.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    selected_points_ros.fields[2].count = 1;

    for( int i = 0; i < num_points; i++ )
    {
        QModelIndex child_index = model->index( i, 0 );
        rviz::Property* child = model->getProp( child_index );
        rviz::VectorProperty* subchild = (rviz::VectorProperty*) child->childAt( 0 );
        Ogre::Vector3 vec = subchild->getVector();

        uint8_t* ptr = &selected_points_ros.data[0] + i * selected_points_ros.point_step;
        *(float*)ptr = vec.x;
        ptr += 4;
        *(float*)ptr = vec.y;
        ptr += 4;
        *(float*)ptr = vec.z;
        ptr += 4;
    }
    selected_points_ros.header.stamp = ros::Time::now();
    /////////////////////////////////////////////////////////////////////////////////////////
    
    // Generating bounding box
    // Convert the ros point cloud message with the selected points into a pcl point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_points_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(selected_points_ros, *selected_points_pcl);

    // Generate an oriented bounding box around the selected points in RVIZ
    // Compute principal direction
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*selected_points_pcl, centroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*selected_points_pcl, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
    eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

    // Move the points to the that reference frame
    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    p2w.block<3,3>(0,0) = eigDx.transpose();
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*selected_points_pcl, cPoints, p2w);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);
    const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    // Final transform and bounding box size
    const Eigen::Quaternionf qfinal(eigDx);
    const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();
    double bb_size_x = max_pt.x - min_pt.x;
    double bb_size_y = max_pt.y - min_pt.y;
    double bb_size_z = max_pt.z - min_pt.z;

    // NOTE: Use these two lines and change the following code (templates on PointXYZ instead of PointXYZRGB)
    // if your input cloud is not colored
    // Convert the point cloud from the callback into a xyz point cloud
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::copyPointCloud(*this->current_pc_, *cloud_xyz);

    // Vectors for the size of the croping box
    Eigen::Vector4f cb_min(-bb_size_x/2.0, -bb_size_y/2.0, -bb_size_z/2.0, 1.0);
    Eigen::Vector4f cb_max(bb_size_x/2.0, bb_size_y/2.0, bb_size_z/2.0, 1.0);

    // We apply the inverse of the transformation of the bounding box to the whole point cloud to boxcrop it (then we do not move the box)
    Eigen::Affine3f transform = Eigen::Translation3f(tfinal)*qfinal;
    Eigen::Affine3f transform_inverse = transform.inverse();

    pcl::CropBox<pcl::PointXYZRGB> crop_filter;
    crop_filter.setTransform(transform_inverse);
    crop_filter.setMax(cb_max);
    crop_filter.setMin(cb_min);
    crop_filter.setKeepOrganized(true);
    crop_filter.setInputCloud(this->current_pc_);

    pcl::PointIndices::Ptr inliers( new pcl::PointIndices() );
    crop_filter.filter(inliers->indices);

    this->selected_segment_pc_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    extract_indices_filter_.reset(new pcl::ExtractIndices<pcl::PointXYZRGB>());
    extract_indices_filter_->setIndices(inliers);
    extract_indices_filter_->setKeepOrganized(true);
    extract_indices_filter_->setInputCloud(this->current_pc_);
    this->selected_segment_pc_->header = this->current_pc_->header;
    extract_indices_filter_->filter(*this->selected_segment_pc_);

    this->num_selected_points_ = inliers->indices.size();

    // ROS_INFO_STREAM_NAMED("SelectedPointsPublisher._processSelectedAreaAndFindPoints",
                        //   "Real number of points of the point cloud in the selected area (NOT published, NOT added): "<< this->num_selected_points_);

    // Publish the bounding box as a rectangular marker
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = context_->getFixedFrame().toStdString().c_str();
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = tfinal.x();
    marker.pose.position.y = tfinal.y();
    marker.pose.position.z = tfinal.z();
    marker.pose.orientation.x = qfinal.x();
    marker.pose.orientation.y = qfinal.y();
    marker.pose.orientation.z = qfinal.z();
    marker.pose.orientation.w = qfinal.w();
    marker.scale.x = max_pt.x - min_pt.x + 0.16;
    marker.scale.y = max_pt.y - min_pt.y + 0.16;
    marker.scale.z = max_pt.z - min_pt.z + 0.16;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration();
    bb_marker_pub_.publish(marker);
    // loading annotation with needed objects such as number of points, the bounding box marker and the pc2 selected data
    annotation_msgs::Annotation newAnnotation;
    newAnnotation.num_points = num_points;
    newAnnotation.bounding_box = marker;
    newAnnotation.captured_point_cloud = selected_points_ros;
    // publishing annotation selected
    annotation_selected_created_publisher.publish(newAnnotation);
    
    return 0;
}
// removes the current selected points and deletes the bounding box 
void SelectedPointsPublisher::removeSelectedPoints(){
    // removing current selection
    rviz::SelectionManager* sel_manager = context_->getSelectionManager();
    rviz::M_Picked selection = sel_manager->getSelection();
    sel_manager->removeSelection(selection);
    // resetting pointcloud segment
    selected_segment_pc_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    // removing current selection bounding box marker
    this->remove_selected_bounding_box_marker();
    std_msgs::Bool annotation_removed;
    // updating annotation removed topic
    annotation_removed.data = true;
    this->annotation_selected_removed_publisher.publish(annotation_removed);
    
    // sel_manager->removeSelection(selection);
    // visualization_msgs::Marker marker;
    // // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    // marker.header.frame_id = context_->getFixedFrame().toStdString().c_str();
    // marker.header.stamp = ros::Time::now();
    // marker.ns = "basic_shapes";
    // marker.id = 0;
    // marker.type = visualization_msgs::Marker::CUBE;
    // marker.action = visualization_msgs::Marker::DELETE;
    // marker.lifetime = ros::Duration();

    // selected_segment_pc_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    // annotation_msgs::Annotation annotation;
    // annotation.bounding_box = marker;
    // annotation_selection_publisher.publish(annotation);
    // bb_marker_pub_.publish(marker);
}
// removes current bounding box marker
void SelectedPointsPublisher::remove_selected_bounding_box_marker(){
    // deleting marker
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = context_->getFixedFrame().toStdString().c_str();
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::DELETE;
    marker.lifetime = ros::Duration();  
    bb_marker_pub_.publish(marker);
}

} // end namespace rviz_plugin_selected_points_topic

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rqt_mypkg::SelectedPointsPublisher, rviz::Tool )