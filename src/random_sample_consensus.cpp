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
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <Eigen/Core>

//added comments


using namespace std::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, 
           const pcl::PointCloud<pcl::Normal>::Ptr& normals){
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->resetCameraViewpoint("sample cloud");
  viewer->initCameraParameters ();
  viewer->resetCameraViewpoint("sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);

  return (viewer);
}
//filtering the point cloud in an axis
void passThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  // min and max values in z axis to keep
  pass.setFilterLimits(0.3, 2.5);
  pass.filter(*cloud);
}
  
/** \brief Given the pointcloud and pointer cloud_normals compute the point normals and store in cloud_normals.
@param cloud - Pointcloud.
@param cloud_normals - The point normals once computed will be stored in this. */
void computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
              const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals){
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setSearchMethod(kdtree);
  ne.setInputCloud(cloud);
  // Set the number of k nearest neighbors to use for the feature estimation.
  ne.setKSearch(50);
  //ne.setViewPoint (float vpx, float vpy, float vpz);
  ne.compute(*cloud_normals);
} 

/** \brief Given the pointcloud and indices of the plane, remove the planar region from the pointcloud.
      @param cloud - Pointcloud.
      @param inliers_plane - Indices representing the plane. */
void removePlaneSurface(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
			const pcl::PointIndices::Ptr& inliers_plane){
  // create a SAC segmenter without using normals
  pcl::SACSegmentation<pcl::PointXYZ> segmentor;
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_PLANE);
  segmentor.setMethodType(pcl::SAC_RANSAC);
  /* run at max 1000 iterations before giving up */
  segmentor.setMaxIterations(1000);
  /* tolerance for variation from model */
  segmentor.setDistanceThreshold(0.01);
  segmentor.setInputCloud(cloud);
  /* Create the segmentation object for the planar model and set all the parameters */
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  segmentor.segment(*inliers_plane, *coefficients_plane);
  /* Extract the planar inliers from the input cloud */
  pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
  extract_indices.setInputCloud(cloud);
  extract_indices.setIndices(inliers_plane);
 
  /* Remove the planar inliers, extract the rest */
  extract_indices.setNegative(true);
  extract_indices.filter(*cloud);
}

/** \brief Given the point normals and point indices, extract the normals for the indices.
      @param cloud_normals - Point normals.
      @param inliers_plane - Indices whose normals need to be extracted. */
void extractNormals(const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                    const pcl::PointIndices::Ptr& inliers_plane,
                    bool negative){
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  extract_normals.setNegative(negative);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*cloud_normals);
}

 /** \brief Given the pointcloud, pointer to pcl::ModelCoefficients and point normals extract the pudding
  from the pointcloud and store the pudding parameters in coefficients_pudding.
      @param cloud - Pointcloud whose plane is removed.
      @param coefficients_pudding - pudding parameters used to define an infinite cylinder will be stored here.
      @param cloud_normals - Point normals corresponding to the plane on which cylinder is kept */
void extractPudding(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                     const pcl::ModelCoefficients::Ptr& coefficients_pudding,
                     const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                     const pcl::PointIndices::Ptr& inliers_pudding){
  // Create the segmentation object for cylinder segmentation and set all the parameters
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segmentor;
  segmentor.setOptimizeCoefficients(true);
  segmentor.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  segmentor.setMethodType(pcl::SAC_RANSAC);
  // Set the normal angular distance weight
  segmentor.setNormalDistanceWeight(0.1);
  // run at max 1000 iterations before giving up
  segmentor.setMaxIterations(1000);
  // tolerance for variation from model
  segmentor.setDistanceThreshold(0.008);
  // min max values of radius in meters to consider
  //segmentor.setRadiusLimits(0.01, 0.1);
  segmentor.setAxis(Eigen::Vector3f (1.0, 1.0, 0.0));
  segmentor.setInputCloud(cloud);
  segmentor.setInputNormals(cloud_normals);

  // Obtain the cylinder inliers and coefficients
  segmentor.segment(*inliers_pudding, *coefficients_pudding);

  // Extract the cylinder inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers_pudding);
  extract.setNegative(false);
  extract.filter(*cloud);
}

 pcl::visualization::PCLVisualizer::Ptr normalsVis (
     pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
 {
   // --------------------------------------------------------
   // -----Open 3D viewer and add point cloud and normals-----
   // --------------------------------------------------------
   pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
   viewer->setBackgroundColor (0.05, 0.05, 0.05);
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 255, 255);
   viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
   //viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
   viewer->addCoordinateSystem (1.0,"cloud",0);
   //viewer->initCameraParameters ();
   return (viewer);
 }

void segmentTable(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& seg_cloud){
  double z_min = -5, z_max = 0;
  
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  // proj.setIndices (inliers);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*seg_cloud);
  /* Create Convex Hull to segment everything above plane */
  pcl::ConvexHull<pcl::PointXYZ> cHull;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndices::Ptr hull_inliers (new pcl::PointIndices);
  cHull.setInputCloud(seg_cloud);
  cHull.reconstruct(*cloud_hull);
  cHull.setDimension (2);
  if (cHull.getDimension () == 2){
    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    prism.setInputCloud (cloud);
    prism.setInputPlanarHull (cloud_hull);
    prism.setHeightLimits (z_min, z_max);
    prism.setHeightLimits (z_min, z_max);
    prism.segment (*hull_inliers);
  }
  pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
  extract_indices.setInputCloud(cloud);
  extract_indices.setIndices(hull_inliers);
  extract_indices.setNegative(true);
  extract_indices.filter(*cloud);
}

geometry_msgs::Quaternion findCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                       const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals){
  geometry_msgs::Quaternion msg;
  geometry_msgs::Quaternion msg2;
  double x_sum = 0, y_sum = 0, z_sum = 0, nx_sum = 0, ny_sum = 0, nz_sum = 0, curv_sum = 0;
  double total = cloud->size(); 
  
  for(int i = 0; i < cloud->height; i++){
    for(int j = 0; j < cloud->width; j++){
      pcl::PointXYZ point = cloud->points[i,j];
      pcl::Normal norm = cloud_normals->points[i,j];
      x_sum = x_sum + point.x;
      y_sum = y_sum + point.y;
      z_sum = z_sum + point.z;
      nx_sum = nx_sum + norm.normal_x;
      ny_sum = ny_sum + norm.normal_y;
      nz_sum = nz_sum + norm.normal_z;
      curv_sum = curv_sum + norm.curvature;
    }
  }   
  msg.x = x_sum / total;
  msg.y = y_sum / total;
  msg.z = z_sum / total;
  msg2.x = nx_sum / total;
  msg2.y = ny_sum / total;
  msg2.z = nz_sum / total;
  msg2.w = curv_sum / total;
  std::cerr << msg << std::endl;
  std::cerr << msg2 << std::endl;  
  return msg; 
}

Eigen::Vector4f find_centroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
  Eigen::Vector4f xyz_centroid;
  compute3DCentroid(*cloud, xyz_centroid);
  return xyz_centroid;
}

Eigen::Vector4f adv_grasp_points(Eigen::Vector4f point){
  
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try
  {
    listener.waitForTransform("/world", "/panda_camera_optical_link", ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform("/world", "/panda_camera_optical_link", ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    
  }

  //looking up transform of link to hand
  tf::TransformListener listener1;
  tf::StampedTransform transform1;
  try
  {
    listener1.waitForTransform("/panda_hand", "/panda_link7", ros::Time(0), ros::Duration(1.0));
    listener1.lookupTransform("/panda_hand", "/panda_link7", ros::Time(0), transform1);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    
  }
  tf::Matrix3x3 R(transform.getRotation());
  Eigen::Matrix4f TWorld2Cam;
  TWorld2Cam << R[0][0], R[0][1], R[0][2], transform.getOrigin().x(),
  R[1][0], R[1][1], R[1][2], transform.getOrigin().y(),
  R[2][0], R[2][1], R[2][2], transform.getOrigin().z(),
  0.0, 0.0, 0.0, 1.0;

  Eigen::Vector4f trans;
  trans= TWorld2Cam*point;
  return trans;
}

int segment_front_surface(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.001);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
  
  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    return (-1);
  }
  
  //using the indices to filter
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
  // Extract the inliers
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cloud);
  std::cerr << "PointCloud representing the planar component: " << cloud->width * cloud->height << " data points." << std::endl;

  return (0);
}

void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  pcl::fromPCLPointCloud2(pcl_pc2,*original_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr seg_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  passThroughFilter(temp_cloud);
  
  segmentTable(temp_cloud, seg_cloud);
  
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(temp_cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*temp_cloud);
  
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  computeNormals(temp_cloud, cloud_normals);
 
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  removePlaneSurface(temp_cloud, inliers_plane);
  
  extractNormals(cloud_normals, inliers_plane, true);
  
  pcl::ModelCoefficients::Ptr coefficients_pudding(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_pudding(new pcl::PointIndices);
  extractPudding(temp_cloud, coefficients_pudding, cloud_normals,inliers_pudding);
  //computeNormals(temp_cloud, cloud_normals);
  extractNormals(cloud_normals, inliers_pudding, false);
  std::cerr << temp_cloud->size() << std::endl;
  std::cerr << cloud_normals->size() << std::endl;

  findCentroid(temp_cloud, cloud_normals);
  
  // just for now segmenting the front surface of the object
  // dont need  
  //pcl::PointCloud<pcl::PointXYZ>::Ptr ind;
  //pcl::copyPointCloud(*temp_cloud, *ind);
  segment_front_surface(temp_cloud);
  Eigen::Vector4f xyz_centroid;
  Eigen::Vector4f trans;
  xyz_centroid = find_centroid(temp_cloud);
  Eigen::Vector4f xyz_centroid2 = Eigen::Vector4f(xyz_centroid[0],xyz_centroid[1], xyz_centroid[2],1);
  trans = adv_grasp_points(xyz_centroid2);

  std::cout << trans << "/n"<<endl;

  // trying to visualize:
  pcl::visualization::PCLVisualizer::Ptr viewer;
  viewer = normalsVis(temp_cloud, cloud_normals);
  while (!viewer->wasStopped())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
  

}

int
main(int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/panda_camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
 }