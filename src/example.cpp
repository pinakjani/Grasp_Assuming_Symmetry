#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>




const sensor_msgs::PointCloud2 downsample(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*input, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);
  return output;

}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  sensor_msgs::PointCloud2 output;
  output = downsample(input);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);

  // pcl::ModelCoefficients coefficients;
  // pcl::PointIndices inliers;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.02);

  seg.setInputCloud (cloud.makeShared ());
  seg.segment (*inliers, *coefficients);

  // Publish the model coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(*coefficients, ros_coefficients);

  //Extract the RANSAC inlier plane 
  pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
  extract_indices.setInputCloud(cloud.makeShared());
  extract_indices.setIndices(inliers);

  // Comment to next two lines to see the segmented table plane
  extract_indices.setNegative(true);
  extract_indices.filter(cloud);

  // Convert back to ROS msg from PCL type
  sensor_msgs::PointCloud2 output1;
  pcl::toROSMsg(cloud,output1);
  //pub.publish (ros_coefficients);
  //pub1.publish(output1);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/panda_camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  //pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);
  //pub1 = nh.advertise<sensor_msgs::PointCloud2> ("output1", 1);

  // Spin
  ros::spin ();
}

// void 
// cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
// {
  
//   // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
//   pcl::fromROSMsg (*input, *cloud);
//   // pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
//   // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//   // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//   // pcl::PCLPointCloud2 cloud_filtered;

//   // Convert to PCL data type
//   // pcl_conversions::toPCL(*input, *cloud);
   
  
//   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//   pcl::PointIndices::Ptr inliers;
//   // Create the segmentation object
//   pcl::SACSegmentation<pcl::PointXYZ> seg;
//   // Optional
//   seg.setOptimizeCoefficients (true);
//   // Mandatory
//   seg.setModelType (pcl::SACMODEL_PLANE);
//   seg.setMethodType (pcl::SAC_RANSAC);
//   seg.setDistanceThreshold (0.01);
 
//   seg.setInputCloud (cloud);
 
  
//   seg.segment (*inliers, *coefficients); 
//   seg.segment (*inliers, *coefficients); 
//   std::cout<<"fine"<<std::endl;
//   pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
  
  
//   extract_indices.setInputCloud(cloud);
//   extract_indices.setIndices(inliers);
//   extract_indices.setNegative(true);
//   extract_indices.filter(*cloud);
  
//   // Publish the model coefficients
//   pcl_msgs::ModelCoefficients ros_coefficients;
//   pcl_conversions::fromPCL(*coefficients, ros_coefficients);
  
//   // pcl::PCLPointCloud2 pcl2;
//   sensor_msgs::PointCloud2 output1;
//   pcl::toROSMsg(*cloud,output1);
//   // pcl::toPCLPointCloud2(cloud, pcl2);
//   // pcl_conversions::fromPCL(pcl2,output1);
  
//   pub.publish (ros_coefficients);
//   pub1.publish (output1);


// }