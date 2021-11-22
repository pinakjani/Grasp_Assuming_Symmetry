#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/common/shapes.h>
#include <pcl/common/centroid.h>

int main ()
{
  // Reading stored pointcloud of pointXYZ in cloud variable
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd4.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout<<"Loaded "<<cloud->width*cloud->height<<" data points from test_pcd.pcd"<< std::endl;


	// Calculating Normals for the read pointcloud
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.03); // Use all neighbors in a sphere of radius 3cm
  ne.compute (*cloud_normals); // Compute the features


  // Finding the Eigen vectors of the pointcloud
	pcl::PCA<pcl::PointXYZ> pca; 
	pca.setInputCloud(cloud); // computed in the constructor
	Eigen::Matrix3f eigen_vectors = pca.getEigenVectors(); // returns computed eigen vectors as a matrix
	std::cout<<"The eigens vectors from PCA are"<<endl;
	std::cout<<eigen_vectors<<endl;
  std::cout<<eigen_vectors(0,0)<<endl;
  // Finding the centroid
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud,centroid);
  std::cout<<"The centroid is "<<endl;
	std::cout<<centroid<<std::endl;


  // Line of first eigen vector
	const pcl::PointXYZ pt1(-eigen_vectors(0,0),-eigen_vectors(0,1),-eigen_vectors(0,2));
	const pcl::PointXYZ pt2(eigen_vectors(0,0),eigen_vectors(0,1),eigen_vectors(0,2));
  // Line of centroid from origin
  const pcl::PointXYZ pt3(0,0,0);
	const pcl::PointXYZ pt4(centroid[0],centroid[1],centroid[2]);


  //Transformation part
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud2 (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud3 (new pcl::PointCloud<pcl::PointXYZ> ());
  // Generate Transformation matrices
  Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  // Angles of rotation
  float theta = 0; 
  Eigen::Vector3f eigenAxis(eigen_vectors(0,0),eigen_vectors(0,1),eigen_vectors(0,2));
  // Move object to origin 
  transform_1.translation() << -centroid[0], -centroid[1], -centroid[2];
  pcl::transformPointCloud (*cloud, *transformed_cloud, transform_1);
  // Rotate object about Y-axis and move object back to original position
  float x = 0;
  transform_2.rotate (Eigen::AngleAxisf (3.14, Eigen::Vector3f::UnitY()));
  transform_2.translation() << 0,0,x;//centroid[0], centroid[1], centroid[2];
  pcl::transformPointCloud (*transformed_cloud, *transformed_cloud2, transform_2);

  Eigen::Vector3f translationAxis(0,1,0);

	//Visualising the cloud. Red->X, Green->Y & Blue->Z , 
	pcl::visualization::PCLVisualizer viewer ("Pointcloud display");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 20, 20, 255);
	viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");

	//visualising the normals of the cloud, camera z-axis towards object and y-axis downwards
	//viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals, 10, 0.05, "normals");
	viewer.addLine<pcl::PointXYZ> (pt3,pt4); //Line showing centroid

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler2 (transformed_cloud2, 230, 20, 20); // Red
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler3 (transformed_cloud2, 20, 230, 20);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler4 (transformed_cloud2, 230, 230, 230);
  viewer.addPointCloud (transformed_cloud2, transformed_cloud_color_handler2, "transformed_cloud");
  viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler3, "transformed_cloud_linear");
	viewer.addPointCloud (transformed_cloud3, transformed_cloud_color_handler4, "transformed_cloud_movingLinear");
  viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
	Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
  while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce(1);
    x += 0.000001; 
    
    //transform_3.rotate (Eigen::AngleAxisf (0, translationAxis));
    transform_3.translation() << 0,0,-x;
    std::cout << transform_3.matrix() << std::endl;
    std::cout <<"------------"<<std::endl;
    pcl::transformPointCloud (*transformed_cloud, *transformed_cloud3, transform_3);
    viewer.updatePointCloud(transformed_cloud3,"transformed_cloud_movingLinear");
  }

  return (0);
}
