#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/pca.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/centroid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>

// This function displays the help
void
showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}

int main (int argc, char** argv)
{
  // Show help
  if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
    showHelp (argv[0]);
    return 0;
  }
  // Fetch point cloud filename in arguments | Works with PCD and PLY files
  std::vector<int> filenames;
  bool file_is_pcd = false;

  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

  if (filenames.size () != 1)  {
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

    if (filenames.size () != 1) {
      showHelp (argv[0]);
      return -1;
    } else {
      file_is_pcd = true;
    }
  }

  // Load file | Works with PCD and PLY files
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  if (file_is_pcd) {
    if (pcl::io::loadPCDFile (argv[filenames[0]], *cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }
  } else {
    if (pcl::io::loadPLYFile (argv[filenames[0]], *cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }
  }

  // Reading stored pointcloud of pointXYZ in cloud variable
  /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../test_pcd3.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout<<"Loaded "<<cloud->width*cloud->height<<"data points from test_pcd3.pcd"<< std::endl;*/
 
  //Finding the Eigen vectors of the original pointcloud
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cloud);
  Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
  // cout<<"eigen_matrix"<<eigen_vectors<<endl;

  // Finding the Centroid of the Cloud
  Eigen::Vector4f goldenMidPt = pca.getMean();
  // std::cout<<goldenMidPt<<endl;

  //Transforming the original cloud to the origin
  Eigen::Matrix4f goldenTransform = Eigen::Matrix4f::Identity();
  goldenTransform.block<3, 3>(0, 0) = eigen_vectors;
  goldenTransform.block<4, 1>(0, 3) = goldenMidPt;
  pcl::PointCloud<pcl::PointXYZ>::Ptr orientedGolden(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud, *orientedGolden, goldenTransform.inverse());
 
  //Finding the Eigen vectors of the new pointcloud at origin using different method to finding each vector
  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  feature_extractor.setInputCloud(orientedGolden);
  feature_extractor.compute();
  feature_extractor.getEigenVectors(major_vector,middle_vector,minor_vector);
  cout<<"origin vector:"<<middle_vector<<major_vector<<minor_vector<<endl;
  
  //Deciding the axis of symmetry through dot product
  Eigen::Vector3f view(0,0,-1);
  Eigen::Vector3f axis;  // The selected axis
  //Eigen::Vector3f rotated_axis;  // The selected rotated axis
  int select_axis = 0;
  float dot1 = abs(view[0]*major_vector[0]+view[1]*major_vector[1]+view[2]*major_vector[2]);
  float dot2 = abs(view[0]*minor_vector[0]+view[1]*minor_vector[1]+view[2]*minor_vector[2]);
  float dot3 = abs(view[0]*middle_vector[0]+view[1]*middle_vector[1]+view[2]*middle_vector[2]);
  if(dot1<dot2 && dot1<dot3)
  {
    axis = major_vector;
    //select_axis = 1;
  } 
  else if(dot2<dot3)
  {
    axis = minor_vector;
    //select_axis = 2;
  } 
  else{
    axis = middle_vector;
    //select_axis = 3;
  }


  //cout<<"idhar dekho:"<<dot1<<" "<<dot2<<" "<<dot3<<endl;
  //cout<<"axis is: "<<axis<<endl;

  // Rotating the pointcloud along the selected axis with theta radians (180 degrees)
  float theta = M_PI;
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.rotate (Eigen::AngleAxisf (theta, axis));
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputpcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*orientedGolden, *outputpcl, transform);


  // Create a set of planar coefficients with X=Y=0,Z=1, projection on front surface
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1;
  coefficients->values[3] = -1;

  // Create a set of planar coefficients with X=0, Y=1,Z=0, projection on top surface
  pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients ());
  coefficients2->values.resize (4);
  coefficients2->values[0] = 0;
  coefficients2->values[1] = 1;
  coefficients2->values[2] = 0;
  coefficients2->values[3] = -1;

  // Create the filtering object z direction
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (orientedGolden);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  // Create the filtering object y direction
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected_y (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ProjectInliers<pcl::PointXYZ> proj3;
  proj3.setModelType (pcl::SACMODEL_PLANE);
  proj3.setInputCloud (orientedGolden);
  proj3.setModelCoefficients (coefficients2);
  proj3.filter (*cloud_projected_y);

  
  float max_y = INT_MIN;
  float max_x = INT_MIN;
  float max_z = INT_MIN;
  float min_z = INT_MAX;
  for (const auto& pointa: *orientedGolden){
    // cout<<"z_max:"<<pointa.z<<endl;
      if(pointa.z>max_z){
        max_z = pointa.z;
      }
      if(pointa.z<min_z){
          min_z = pointa.z;
      }
      if(pointa.y>max_y)
        max_y = pointa.y;
      if(pointa.x>max_x)
        max_x = pointa.x;
    }
  cout<<"MAX_Z:"<<max_z<<endl;
  cout<<"MIN_Z:"<<min_z<<endl;

  // Rotated point cloud on origin - y value
  float max_y2 = INT_MIN;
  float max_x2 = INT_MIN;
  float max_z2 = INT_MIN;
  float min_z2 = INT_MAX;
  // float max_y2 = INT_MIN;
  for (const auto& pointn: *outputpcl){
      // cout<<"y2_max_origin:"<<pointn.y<<endl;
      if(pointn.y>max_y2)
        max_y2 = pointn.y;
      if(pointn.z>max_z2)
        max_z2 = pointn.z;
      if(pointn.z<min_z2)
        min_z2 = pointn.z;
      if(pointn.x>max_x2)
        max_x2 = pointn.x;     
  }

 
    cout<<"MAX_x:"<<max_x<<endl;
    cout<<"MAX_y:"<<max_y<<endl;
    cout<<"MAX_x2:"<<max_x2<<endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr offsetcloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr rot_cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr rot_cloud_projected_y (new pcl::PointCloud<pcl::PointXYZ>);
  //Visualisation of the cloud
  pcl::visualization::PCLVisualizer viewer ("Simple pointcloud display example");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 255, 20, 25); //Red
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler2 (orientedGolden, 20, 20, 255); //Blue
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler3 (outputpcl, 20, 255, 20); //Green
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler4 (offsetcloud, 200, 200, 200); // White
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_projected (cloud_projected, 200, 200, 0); // Orange
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_projected_x (cloud_projected_y, 100, 100, 0); // Orange
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_rot_projected (rot_cloud_projected, 200, 0, 100); // RedBlue
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_rot_projected_x (rot_cloud_projected_y, 100, 0, 100); // RedBlue
  viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");
  viewer.addPointCloud (orientedGolden, source_cloud_color_handler2, "original2_cloud");
  viewer.addPointCloud (outputpcl, source_cloud_color_handler3, "original3_cloud");
  viewer.addPointCloud (offsetcloud, source_cloud_color_handler4, "original4_cloud");
  viewer.addPointCloud (cloud_projected, source_cloud_projected, "projected_cloud");
  viewer.addPointCloud (cloud_projected_y, source_cloud_projected_x, "projected_cloud_y");
  viewer.addPointCloud (rot_cloud_projected, source_cloud_rot_projected, "rot_projected_cloud");
  viewer.addPointCloud (rot_cloud_projected_y, source_cloud_rot_projected_x, "rot_projected_cloud_y");
  // viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals, 10, 0.05, "normals");
  viewer.addCoordinateSystem (0.5, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  // viewer.addSphere(coeff);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original2_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original3_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original4_cloud");
  
  // Logic for translation and rotation of selected axis
  int count=0;
  float offset = 0;
  float max_offset = 0;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> symmetry_clouds;
  if (max_z2>max_z)
        max_offset = (-(max_z-min_z));
    else{
        max_offset = max_z-min_z;
    }
  theta = 0;//-1*((20*M_PI)/180);
  Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce (1);  
    while(count<1){
      if(max_offset<0 && offset>max_offset){
        offset += -0.0001;
      } else if(max_offset>0 && offset<max_offset){
        offset += 0.0001;
      } else{
        count=1;
        cout<<"We here folks!! size:"<<symmetry_clouds.size()<<endl;
        break;
      }
      if(theta==0){
        transform2.rotate (Eigen::AngleAxisf ((-20*M_PI/180), Eigen::Vector3f::UnitY()));
        pcl::transformPointCloud(*outputpcl, *offsetcloud, transform2);
        viewer.updatePointCloud(offsetcloud,"original4_cloud");
        theta = 1;
      }
      while(theta <= 40){
      transform2.rotate (Eigen::AngleAxisf ((1*M_PI/180), Eigen::Vector3f::UnitY()));
      pcl::transformPointCloud(*outputpcl, *offsetcloud, transform2);
      viewer.updatePointCloud(offsetcloud,"original4_cloud");
      symmetry_clouds.push_back(offsetcloud);
      //cout<<theta<<"   "<<count<<endl;
      theta++;
      }
      cout<<theta<<"   "<<count<<" size:"<<symmetry_clouds.size()<<endl;
      transform2.rotate (Eigen::AngleAxisf ((-40*M_PI/180), Eigen::Vector3f::UnitY()));
      transform2.translation()<<0,-1*(max_y2-max_y),offset;
      pcl::transformPointCloud(*outputpcl, *offsetcloud, transform2);
      viewer.updatePointCloud(offsetcloud,"original4_cloud");
      theta = 1;
    }
  }
  return (0);
}






    // if(max_offset<0 && offset>max_offset){
    //     offset += -0.00001;
    // } else if(max_offset>0 && offset<max_offset){
    //     offset += 0.00001;
    // } else{
    //     count=1;
    //     cout<<"We here folks!!"<<endl;
    // }
    // if(count==0){



      // pcl::ProjectInliers<pcl::PointXYZ> rot_proj;
      // rot_proj.setModelType (pcl::SACMODEL_PLANE);
      // rot_proj.setInputCloud (offsetcloud);
      // rot_proj.setModelCoefficients (coefficients);
      // rot_proj.filter (*rot_cloud_projected);
      // viewer.updatePointCloud(rot_cloud_projected,"rot_projected_cloud");

      // pcl::ProjectInliers<pcl::PointXYZ> rot_proj_x;
      // rot_proj_x.setModelType (pcl::SACMODEL_PLANE);
      // rot_proj_x.setInputCloud (offsetcloud);
      // rot_proj_x.setModelCoefficients (coefficients2);
      // rot_proj_x.filter (*rot_cloud_projected_y);
      // viewer.updatePointCloud(rot_cloud_projected_y,"rot_projected_cloud_y");