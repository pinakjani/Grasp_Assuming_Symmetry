#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/pca.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/centroid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>

int main ()
{
  // Reading stored pointcloud of pointXYZ in cloud variable
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../test_pcd3.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout<<"Loaded "<<cloud->width*cloud->height<<"data points from test_pcd3.pcd"<< std::endl;
 
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
  if (max_z2>max_z)
        max_offset = (-(max_z-min_z));
    else{
        max_offset = max_z-min_z;
    }
  theta = 0;//-1*((20*M_PI)/180);
  Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce (1);  
    if(max_offset<0 && offset>max_offset){
        offset += -0.00001;
    } else if(max_offset>0 && offset<max_offset){
        offset += 0.00001;
    } else{
        count=1;
        cout<<"We here folks!!"<<endl;
    }
    transform2.translation()<<0,0,offset;
    theta = 0;
    while (theta<(20) && count==0){
      // cout<<"in"<<endl;
      transform2.rotate (Eigen::AngleAxisf ((theta*M_PI/180), Eigen::Vector3f::UnitY()));
      // std::cout<<transform2.matrix() <<std::endl;
      // std::cout<<"-----------------"<<std::endl;
      
      pcl::transformPointCloud(*outputpcl, *offsetcloud, transform2);
      viewer.updatePointCloud(offsetcloud,"original4_cloud");
      
      pcl::ProjectInliers<pcl::PointXYZ> rot_proj;
      rot_proj.setModelType (pcl::SACMODEL_PLANE);
      rot_proj.setInputCloud (offsetcloud);
      rot_proj.setModelCoefficients (coefficients);
      rot_proj.filter (*rot_cloud_projected);
      viewer.updatePointCloud(rot_cloud_projected,"rot_projected_cloud");

      pcl::ProjectInliers<pcl::PointXYZ> rot_proj_x;
      rot_proj_x.setModelType (pcl::SACMODEL_PLANE);
      rot_proj_x.setInputCloud (offsetcloud);
      rot_proj_x.setModelCoefficients (coefficients2);
      rot_proj_x.filter (*rot_cloud_projected_y);
      viewer.updatePointCloud(rot_cloud_projected_y,"rot_projected_cloud_y");

      theta+= 1;
    }
    pcl::transformPointCloud(*outputpcl, *offsetcloud, transform2);
    viewer.updatePointCloud(offsetcloud,"original4_cloud");
  }
  return (0);
}



// ------- Calculating Normals for the read pointcloud --------
// pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
// ne.setInputCloud (cloud);
// // Create an empty kdtree representation, and pass it to the normal estimation object.
// // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
// ne.setSearchMethod (tree);
// // Output datasets
// pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
// // Use all neighbors in a sphere of radius 3cm
// ne.setRadiusSearch (0.03);
// // Compute the features
// ne.compute (*cloud_normals);
// std::cout<<cloud_normals->size();
// ------------------------------------------------------------

// -------------- Another method to get the centroid ----------
// Eigen::Vector4f cent;
// pcl::compute3DCentroid(*orientedGolden,cent);
// cout<<"dekho:"<<cent[0];
// cout<<"dekho1:"<<cent-origin<<endl;
// pcl::ModelCoefficients coeff;
// coeff.values.push_back(cent[0]);
// coeff.values.push_back(cent[1]);
// coeff.values.push_back(cent[2]);
// coeff.values.push_back(0.001);
// ------------------------------------------------------------

// ----------- Logic to find common points -------------
// int val = 0;
// for(auto pointc1:*orientedGolden){
//     for(auto pointc2:*offsetcloud){
//       // std::vector<float> vec1={pointc1.x,pointc1.y,pointc1.z},vec2={pointc2.x,pointc2.y,pointc2.z};
        
//       if(pointc1.x==pointc2.x && pointc1.y==pointc2.y){
//         cout<<"dekh:"<<pointc1.x<<" "<<pointc2.x<<endl;
//         val++;
//         break;
//       }

//         // cout<<"hi:"<<pointc1.z<<" "<<pointc2.z<<endl;
//     }
// }
// cout<<"The Common points are:"<<val<<endl;
// ------------------------------------------------------------

// ----------- Logic to rotate points of a point cloud -------------
// theta = 0;
// while(theta<((20*M_PI)/180) && offset<0.05){
//   cout<<"in"<<endl;
//   transform2.rotate (Eigen::AngleAxisf (theta, minor_vector));
//   std::cout<<transform2.matrix() <<std::endl;
//   std::cout<<"-----------------"<<std::endl;    
//   cout<<"The Common points are:"<<val<<endl;
//   pcl::transformPointCloud(*outputpcl, *offsetcloud, transform2);
//   viewer.updatePointCloud(offsetcloud,"original4_cloud");
//   theta+= (M_PI/180);
// }
// ------------------------------------------------------------



/* -------------------- Pinak's Height logic ------------------
    // Point cloud on origin - z max value
    float max_z = INT_MIN;
    for (const auto& pointa: *orientedGolden){
        //cout<<"z_max:"<<pointa.z<<endl;
        if(pointa.z>max_z)
        {
        max_z = pointa.z;
        }
    }
    //cout<<"MAX_Z:"<<max_z<<endl;

    // Point cloud on origin - z min value
    float min_z = INT_MAX;
    for (const auto& pointb: *orientedGolden){
        //cout<<"z_min:"<<pointb.z<<endl;
        if(pointb.z<min_z){
            min_z = pointb.z;
        }
    }
    //cout<<"MIN_Z:"<<min_z<<endl;

    // Rotated point cloud on origin - y value
    float max_y2 = INT_MIN;
    for (const auto& pointn: *outputpcl){
        //cout<<"y2_max_origin:"<<pointn.y<<endl;
        if(pointn.y>max_y2)
        max_y2 = pointn.y;

    }

    // Point cloud on origin - y value
    float max_y = INT_MIN;
    for (const auto& point: *orientedGolden){
        //cout<<"y_max_origin:"<<point.y<<endl;
        if(point.y>max_y)
        max_y = point.y;
    }

    //cout<<"MAX_Y"<<max_y<<" "<<max_y2;

    // Rotated point cloud on origin - x value
    float max_x2 = INT_MIN;
    for (const auto& pointn: *orientedGolden){
        //cout<<"x_n:"<<pointn.x<<endl;
        if(pointn.x>max_x2)
        max_x2 = pointn.x;

    }

    // Point cloud on origin - x value
    float max_x = INT_MIN;
    for (const auto& point: *orientedGolden){
        //cout<<"x:"<<point.x<<endl;
        if(point.x>max_x)
        max_x = point.x;
    }

    //cout<<"MAX_X"<<max_x<<" "<<max_x2;

*/
// -----------------------------------------------------------------------


/* ---------------------Angle between rotated point clouds logic ------------------
  // Finding the Eigen vectors of the rotated pointcloud at origin using different method to finding each vector
  Eigen::Vector3f rotated_major_vector, rotated_middle_vector, rotated_minor_vector;
  feature_extractor.setInputCloud(outputpcl);
  feature_extractor.compute();
  feature_extractor.getEigenVectors(rotated_major_vector,rotated_middle_vector,rotated_minor_vector);
  cout<<"rotated vector:"<<middle_vector<<major_vector<<minor_vector<<endl;

  // Finding angle between eigen vectors at the origin
  double dot_product =0;
  double mag1 =0;
  double mag2 =0;
  switch (select_axis)
  {
  case 1:
    dot_product = major_vector[0]*rotated_major_vector[0]
    +major_vector[1]*rotated_major_vector[1]
    +major_vector[2]*rotated_major_vector[2];
    for (int i = 0; i < 3; i++)
        mag1+= major_vector[i] * major_vector[i];
    mag1 = sqrt(mag1);
    for (int i = 0; i < 3; i++)
        mag2+= rotated_major_vector[i] * rotated_major_vector[i];
    mag2 = sqrt(mag2);
    rotated_axis = rotated_major_vector;
    break;
  case 2:
    dot_product = minor_vector[0]*rotated_minor_vector[0]
    +minor_vector[1]*rotated_minor_vector[1]
    +minor_vector[2]*rotated_minor_vector[2];
    for (int i = 0; i < 3; i++)
        mag1+= minor_vector[i] * minor_vector[i];
    mag1 = sqrt(mag1);
    for (int i = 0; i < 3; i++)
        mag2+= rotated_minor_vector[i] * rotated_minor_vector[i];
    mag2 = sqrt(mag2);
    rotated_axis = rotated_major_vector;
    break;
  case 3:
    dot_product = middle_vector[0]*rotated_middle_vector[0]
    +middle_vector[1]*rotated_middle_vector[1]
    +middle_vector[2]*rotated_middle_vector[2];
    for (int i = 0; i < 3; i++)
        mag1+= middle_vector[i] * middle_vector[i];
    mag1 = sqrt(mag1);
    for (int i = 0; i < 3; i++)
        mag2+= rotated_middle_vector[i] * rotated_middle_vector[i];
    mag2 = sqrt(mag2);
    rotated_axis = rotated_major_vector;
    break;
  default:
    cout<<"Your code just broke, mate!";
    break;
  }
  //cout<<"rotated axis"<<rotated_axis<<endl;
  float angle_between_vectors = dot_product/ (mag1*mag2);
  */

 // cout<<"Angle: "<<angle_between_vectors<<endl;
    // cout<<"The selected axis values: "<<axis<<endl;
    // cout<<"The rotated axis values: "<<rotated_axis<<endl;
 // ---------------------------------------------------------------