#include <iostream>
#include <thread>
#include <ros/ros.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <string>
#include <ostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <geometry_msgs/Quaternion.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> CloudType;

void main(int argc, char** argv){

  ros::init(argc, argv, "pcd_tfs");
  ros::NodeHandle node;
  
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  ros::Rate rate(10.0);
  try{
    transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1", ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    continue;
  }
  
  // Load the PCD files
  CloudType::Ptr cloud1(new CloudType);
  CloudType::Ptr cloudR(new CloudType);
  CloudType::Ptr cloudL(new CloudType);
  pcl::io::loadPCDFile("view1_bonus.pcd", *cloud1);
  pcl::io::loadPCDFile("viewRight_bonus.pcd", *cloudR);
  pcl::io::loadPCDFile("viewLeft_bonus.pcd", *cloudL);

  pcl::Matrix4 Tro;
  pcl::Matrix4 Tlo;  

 

  Tro = [0.5898, 0.6668, 0.4555, -0.1254,
               -0.7177, 0.6914, -0.0830, 0.5133,
               -0.3703, -0.2780, 0.8863, 0.2394,
               0, 0, 0, 1];

  Tlo = [0.3753, -0.8227, 0.4270, -0.01163,
               0.8503, 0.4890, 0.1948, -0.5639,
               -0.3691, 0.2900, 0.8830, 0.2555,
               0, 0, 0, 1];

  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.transformCloud(cloudR, cloud1, Tro);
  //icp.setInputTarget(cloud1);
  
  //CloudType FinalR;
  //icp.align(FinalR);

  pcl::IterativeClosestPoint<PointType, PointType> icpL;
  icpL.setInputSource(cloudL);
  icpL.setInputTarget(cloud1);
  
  CloudType FinalL;
  icpL.align(FinalL);

  // Put in one output cloud
  CloudType::Ptr output(new CloudType);
  *output += *cloud1;
  *output += *cloudR;
  *output += *cloudL;

  // Save the output file
  pcl::io::savePCDFileASCII("output.pcd", *output);
}
