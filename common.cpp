#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/surface/poisson.h>
#include <pcl/common/common.h>

#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/surface/mls.h>

#include <pcl/filters/passthrough.h>


int main ()
{
  // Reading stored pointcloud of pointXYZ in cloud variable
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout<<"Loaded "<<cloud->width*cloud->height<<"data points from test_pcd.pcd"<< std::endl;

  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cloud);
  Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
  std::cout<<eigen_vectors<<std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr orientedGolden(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Vector4f goldenMidPt = pca.getMean();
  Eigen::Matrix4f goldenTransform = Eigen::Matrix4f::Identity();
  goldenTransform.block<3, 3>(0, 0) = eigen_vectors; //read
  goldenTransform.block<4, 1>(0, 3) = goldenMidPt;
  pcl::transformPointCloud(*cloud, *orientedGolden, goldenTransform.inverse());
  std::cout<<goldenMidPt<<endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputpcl(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Matrix3f rotation;
  rotation << -1,0,0,
               0,1,0,
               0,0,-1;
//   pcl::PCA<pcl::PointXYZ> Tpca;
//   Tpca.setInputCloud(orientedGolden);
//   Eigen::Vector4f origin = Tpca.getMean();
//   cout<<"hey:"<<origin<<endl;
  Eigen::Vector4f origin(0.03-5.03785e-07,-7.88388e-08,0.03-9.41368e-07,1);//hard-coded?
//   Eigen::Vector4f origin = goldenTransform.getMean();
  Eigen::Matrix4f pclTransform = Eigen::Matrix4f::Identity();
  pclTransform.block<3, 3>(0, 0) = rotation;
  pclTransform.block<4, 1>(0, 3) = origin;
  pcl::transformPointCloud(*orientedGolden, *outputpcl, pclTransform.inverse());

  Eigen::Vector4f cent;
  pcl::compute3DCentroid(*outputpcl,cent);
  cout<<"dekho:"<<cent;
//   cout<<"dekho1:"<<cent-origin<<endl;

  // Calculating Normals for the read pointcloud
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);
  // Compute the features
  ne.compute (*cloud_normals);
  std::cout<<cloud_normals->size();

  //visualising the cloud
  pcl::visualization::PCLVisualizer viewer ("Simple pointcloud display example");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler3 (outputpcl, 20, 255, 20);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler2 (orientedGolden, 20, 20, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 255, 20, 25);
  viewer.addPointCloud (outputpcl, source_cloud_color_handler3, "original3_cloud");
  viewer.addPointCloud (orientedGolden, source_cloud_color_handler2, "original2_cloud");
  viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");
//   viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals, 10, 0.05, "normals");
      std::cout << "combine points and normals" << std::endl;
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
concatenateFields(*cloud, *cloud_normals, *cloud_smoothed_normals);
      std::cout << "begin poisson reconstruction" <<std::endl;
      pcl::Poisson<pcl::PointNormal> poisson;
      poisson.setDepth(9);
      poisson.setInputCloud(cloud_smoothed_normals);
      pcl::PolygonMesh mesh;
      poisson.reconstruct(mesh);

      pcl::io::savePLYFile("saved.ply", mesh);

  viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original_cloud");
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }

  //visualising the normals of the cloud
  return (0);
}