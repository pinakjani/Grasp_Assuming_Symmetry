#include <iostream>
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
#include <chrono>
#include <thread>

int viewer_script (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> simulation_clouds)
{
int i =0;
std::string path= "/home/pinak/PCL/PCD/test";
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr ref (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr offsetcloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    // Reading stored pointcloud of pointXYZ in cloud variable
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/wael/dr_project/Grasp_Assuming_Symmetry/build/testout.pcd", *ref) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test3.pcd \n");
        return (-1);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/wael/dr_project/Grasp_Assuming_Symmetry/test_pcd.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test3.pcd \n");
        return (-1);
    }

    std::cout<<"Loaded "<<offsetcloud->width*offsetcloud->height<<"data points from test_pcd3.pcd"<< std::endl;
    pcl::visualization::PCLVisualizer viewer ("Simple pointcloud display example");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ref_cloud_color_handler (ref, 255, 25, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 255, 20, 25);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler4 (offsetcloud, 10, 255, 20);
    viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");
    viewer.addPointCloud (ref, ref_cloud_color_handler, "ref_cloud");
    viewer.addPointCloud (offsetcloud, source_cloud_color_handler4, "original4_cloud");
    // viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals, 10, 0.05, "normals");
    viewer.addCoordinateSystem (0.5, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    // viewer.addSphere(coeff);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ref_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original4_cloud");
    
    int count=0;
    float offset = 0;

    while (!viewer.wasStopped ()) {
        viewer.spinOnce(2);
        if(i<simulation_clouds.size()){
            offsetcloud = simulation_clouds[i];
            cout<<"PCD"+std::to_string(i)<<endl;
            viewer.updatePointCloud(offsetcloud,"original4_cloud");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            i++;
        }

 }
return (0);
}



// This function displays the help
void showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}

int main (int argc, char** argv)
{
  //-------------------- Reading the input cloud from console start --------------------//
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
  //-------------------- Reading the input cloud from console end ----------------------//

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
  //cout<<"origin vector:"<<middle_vector<<major_vector<<minor_vector<<endl;

  //Deciding the axis of symmetry through dot product
  Eigen::Vector3f view(0,0,-1);
  Eigen::Vector3f axis;  // The selected axis
  //Eigen::Vector3f rotated_axis;  // The selected rotated axis
  //int select_axis = 0;
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

  // Oriented Golden point cloud on origin 
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
    
  pcl::PointCloud<pcl::PointXYZ>::Ptr rot_cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr rot_cloud_projected_y (new pcl::PointCloud<pcl::PointXYZ>);
  
  //Storage for all hypothesis clouds
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> symmetry_clouds; 
  
  // Logic for translation and rotation of selected axis
  int count=0;
  float offset = 0;
  float max_offset = 0;
  float value=0;
  if (max_z2>max_z)
        max_offset = (-(max_z-min_z));
    else{
        max_offset = max_z-min_z;
    }
  theta = 0; // Unused at the moment
  symmetry_clouds.push_back(outputpcl); // Storing the original rotated cloud in first place
  
  while(1)
  {
    if(max_offset<0 && offset>max_offset){
      offset += -0.001; // offset condition to exit loop
      value = -0.001; // Offset value to move cloud 
    } else if(max_offset>0 && offset<max_offset){
      offset += 0.001; // offset condition to exit loop
      value = 0.001; // Offset value to move cloud 
    } else{
      cout<<"We here folks!! size:"<<symmetry_clouds.size()<<endl;
      break;
    }
    count =0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transfercloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Works when having different point clouds
    if(count==0){
      Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
      transform2.translation()<<0,0,0;
      pcl::PointCloud<pcl::PointXYZ>::Ptr offsetcloud(new pcl::PointCloud<pcl::PointXYZ>);
      transform2.rotate (Eigen::AngleAxisf ((-20*(M_PI/180)), Eigen::Vector3f::UnitY()));
      pcl::transformPointCloud(*outputpcl, *offsetcloud, transform2);
      cout<<"first point of rotation"<<endl; 
      std::cout << transform2.matrix() << std::endl;
      symmetry_clouds.push_back(offsetcloud);
      transfercloud = offsetcloud;
      count++;
    }
    //Main loop, rotates cloud from -20 to +20
    while(count<=40){
      int j=0;
      Eigen::Affine3f transform3 = Eigen::Affine3f::Identity();
      transform3.translation()<<0,0,0;
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      transform3.rotate (Eigen::AngleAxisf (count*(M_PI/180), Eigen::Vector3f::UnitY())); 
      //Continues rotation from previous value of -20, values are added of increasing degrees 1,2,3..
      pcl::transformPointCloud(*transfercloud, *temp_cloud, transform3);
      std::cout << transform3.matrix() << std::endl;
      //if(count==1)
      symmetry_clouds.push_back(temp_cloud);
      cout<<count<<" size"<<symmetry_clouds.size()<<endl;
      count++;
    }
    // Performs translation based on the step size and updates outputpcl
    Eigen::Affine3f transform4 = Eigen::Affine3f::Identity();
    transform4.translation()<<0,0,value;
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*outputpcl, *trans_temp_cloud, transform4);
    pcl::io::savePCDFileASCII ("beforeT", *outputpcl);
    outputpcl = trans_temp_cloud;  // Under Scrutiny
    pcl::io::savePCDFileASCII ("afterT", *outputpcl);
    symmetry_clouds.push_back(outputpcl);
    cout<<count<<" size"<<symmetry_clouds.size()<<endl;
  }

  pcl::io::savePCDFileASCII ("testout.pcd", *symmetry_clouds[0]); //outputpcl different clouds working
  pcl::io::savePCDFileASCII ("test1.pcd", *symmetry_clouds[1]); //-20 offsetcloud1 different clouds working
  pcl::io::savePCDFileASCII ("test2.pcd", *symmetry_clouds[2]); // -19 offsetcloud2 different clouds working
  pcl::io::savePCDFileASCII ("test3.pcd", *symmetry_clouds[3]); // -18 offsetcloud2 same clouds failing, first value of -19 stored
  pcl::io::savePCDFileASCII ("test4.pcd", *symmetry_clouds[4]); // -17 offsetcloud2 same clouds failing
  pcl::io::savePCDFileASCII ("test21.pcd", *symmetry_clouds[21]); // 0 test21 and test1 are similar (only last 3 digits change)
  pcl::io::savePCDFileASCII ("test42.pcd", *symmetry_clouds[42]); 
  pcl::io::savePCDFileASCII ("test43.pcd", *symmetry_clouds[43]); 
  pcl::io::savePCDFileASCII ("testlast2.pcd", *symmetry_clouds[symmetry_clouds.size()-2]);
  pcl::io::savePCDFileASCII ("testlast.pcd", *symmetry_clouds[symmetry_clouds.size()-1]);
  
  int lol = viewer_script(symmetry_clouds);
  cout<<"-----------This is "<<lol<<endl;

  // Selecting which cloud to view and save
  pcl::PointCloud<pcl::PointXYZ>::Ptr viewer_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  int cloud_number=0;
  cout<<"Enter the cloud number"<<endl;
  cin>>cloud_number;
  viewer_cloud = symmetry_clouds[cloud_number];
  pcl::io::savePCDFileASCII ("special.pcd", *viewer_cloud);

  //Visualisation of the cloud
  pcl::visualization::PCLVisualizer viewer ("Simple pointcloud display example");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> symmetry_clouds_color_handlertest (symmetry_clouds[0], 255, 20, 25); //Red
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> symmetry_clouds_color_handler1 (symmetry_clouds[1], 20, 20, 255); //Blue
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> symmetry_clouds_color_handler2 (symmetry_clouds[2], 20, 255, 20); //           Green
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> symmetry_clouds_color_handler3 (symmetry_clouds[3], 200, 200, 200); //         White
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> oriented_golden_color_handler (orientedGolden, 200, 200, 0); //           Orange
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> symmetry_clouds_color_handler21 (symmetry_clouds[21], 200, 200,200); // White
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> symmetry_clouds_color_handlerl2 (viewer_cloud, 20, 255, 0); // Green
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> symmetry_clouds_color_handlerl (symmetry_clouds[symmetry_clouds.size()-1], 20, 255, 0); // Green
  
  viewer.addPointCloud (symmetry_clouds[0], symmetry_clouds_color_handlertest, "original_cloud");
  viewer.addPointCloud (symmetry_clouds[1], symmetry_clouds_color_handler1, "original2_cloud");
//   viewer.addPointCloud (symmetry_clouds[2], symmetry_clouds_color_handler2, "original3_cloud");
//   viewer.addPointCloud (symmetry_clouds[3], symmetry_clouds_color_handler3, "original4_cloud");
  viewer.addPointCloud (orientedGolden, oriented_golden_color_handler, "original4_cloud");
  //viewer.addPointCloud (symmetry_clouds[21], symmetry_clouds_color_handler21, "original3_cloud");
  viewer.addPointCloud (viewer_cloud, symmetry_clouds_color_handlerl2, "original5_cloud");
  //viewer.addPointCloud (symmetry_clouds[symmetry_clouds.size()-1], symmetry_clouds_color_handlerl, "original4_cloud");
  // viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals, 10, 0.05, "normals");
  viewer.addCoordinateSystem (0.5, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  // viewer.addSphere(coeff);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original2_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original5_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original4_cloud");

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce (1); 
    // for(int i=5;i<35;i++){
    //     viewer_cloud = symmetry_clouds[i];
    //     viewer.updatePointCloud(viewer_cloud,"original5_cloud");
    // }
    
  }
  
  return (0);
}