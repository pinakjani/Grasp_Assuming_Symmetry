#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>

int main ()
{
  // Reading stored pointcloud of pointXYZ in cloud variable
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/pinak/PCL/test_pcd4.pcd", *cloud) == -1) //* load the file
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
  cout<<"vector:"<<major_vector<<middle_vector<<minor_vector<<endl;
 
  //Deciding the axis of symmetry through dot product
  Eigen::Vector3f view(0,0,-1);
  Eigen::Vector3f axis;  // The selected axis
  float dot1 = abs(view[0]*major_vector[0]+view[1]*major_vector[1]+view[2]*major_vector[2]);
  float dot2 = abs(view[0]*minor_vector[0]+view[1]*minor_vector[1]+view[2]*minor_vector[2]);
  float dot3 = abs(view[0]*middle_vector[0]+view[1]*middle_vector[1]+view[2]*middle_vector[2]);
  if(dot1<dot2 && dot1<dot3)
    axis = major_vector;
  else if(dot2<dot3)
    axis = minor_vector;
  else
    axis = middle_vector;
  cout<<"idhar dekho:"<<dot1<<" "<<dot2<<" "<<dot3<<endl;

  // Rotating the pointcloud along the selected axis with theta radians
  float theta = M_PI;
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.rotate (Eigen::AngleAxisf (theta, axis));
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputpcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*orientedGolden, *outputpcl, transform);

    // Point cloud on origin - z max value
    // Point cloud on origin - z min value
    float max_z = INT_MIN;
    float min_z = INT_MAX;
    float max_y = INT_MIN;
    float max_x = INT_MIN;
    for (const auto& pointa: *orientedGolden){
        // cout<<"z_max:"<<pointa.z<<endl;
        if(pointa.z>max_z)
        {
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




  float offset = 0;
  if (max_z2>max_z)
    offset = (-(max_z-min_z));
  else{
    offset = max_z-min_z; // break;
  }
  Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
  pcl::PointCloud<pcl::PointXYZ>::Ptr offsetcloud(new pcl::PointCloud<pcl::PointXYZ>);
  theta = 10;
  // -1*(max_y2-max_y)
  transform2.translation()<< 0,-1*(max_y2-max_y),offset+0.01;
  transform2.rotate (Eigen::AngleAxisf ((theta*M_PI/180), middle_vector));
  pcl::transformPointCloud(*outputpcl, *offsetcloud, transform2);

  Eigen::Vector3f offmajor_vector, offmiddle_vector, offminor_vector;
  feature_extractor.setInputCloud(offsetcloud);
  feature_extractor.compute();
  feature_extractor.getEigenVectors(offmajor_vector,offmiddle_vector,offminor_vector);

  float max_x3 = INT_MIN;
  float max_y3 = INT_MIN;
  float max_z3 = INT_MIN;
  float min_z3 = INT_MAX;
  for (const auto& pointb: *offsetcloud){
      // cout<<"y2_max_origin:"<<pointn.y<<endl;
      if(pointb.y>max_y3)
        max_y3 = pointb.y;
      if(pointb.z>max_z3)
        max_z3 = pointb.z;
      if(pointb.z<min_z3)
        min_z3 = pointb.z;
      if(pointb.x>max_x3)
        max_x3 = pointb.x;
      
  }
  // cout<<"MAX_x3:"<<max_x3<<" "<<max_y3<<" "<<max_z3<<endl;
  // Eigen::Affine3f transform3 = Eigen::Affine3f::Identity();
  // transform3.translation()<< 0,0,0;
  // pcl::transformPointCloud(*offsetcloud, *offsetcloud, transform3);

  cout<<"Eigen_offset:"<<offmajor_vector<<" "<<offmiddle_vector<<" "<<offminor_vector<<endl;
  float mag_middle = sqrt(middle_vector[0]*middle_vector[0]+middle_vector[1]*middle_vector[1]+middle_vector[2]*middle_vector[2]);
  float mag_offmiddle = sqrt(offmiddle_vector[0]*offmiddle_vector[0]+offmiddle_vector[1]*offmiddle_vector[1]+offmiddle_vector[2]*offmiddle_vector[2]);
  float dot_prod = middle_vector[0]*offmiddle_vector[0] + middle_vector[1]*offmiddle_vector[1] + middle_vector[2]*offmiddle_vector[2];
  float ang = dot_prod/(mag_middle*mag_offmiddle);
  cout<<"angle:"<<ang<<endl;
  //Visualisation of the cloud
  pcl::visualization::PCLVisualizer viewer ("Simple pointcloud display example");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 255, 20, 25); //Red
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler2 (orientedGolden, 20, 20, 255); //Blue
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler3 (outputpcl, 20, 255, 20); //Green
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler4 (offsetcloud, 200, 200, 200); // White
  viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");
  viewer.addPointCloud (orientedGolden, source_cloud_color_handler2, "original2_cloud");
  // viewer.addPointCloud (outputpcl, source_cloud_color_handler3, "original3_cloud");
  viewer.addPointCloud (offsetcloud, source_cloud_color_handler4, "original4_cloud");
  // viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals, 10, 0.05, "normals");
  // viewer.addCoordinateSystem (0.5, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  // viewer.addSphere(coeff);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original2_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original3_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original4_cloud");
 
  // Logic for translation and rotation of selected axis
  // theta = 180;
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
     
    // while(theta<330){
    //   cout<<"in"<<endl;
    //   transform2.rotate (Eigen::AngleAxisf ((theta*M_PI/180), middle_vector));
    //   // transform2.translation()<< (max_x2-max_x),-1*(max_y2-max_y),offset;
    //   std::cout<<transform2.matrix() <<std::endl;
    //   std::cout<<"-----------------"<<std::endl;     
    //   // cout<<"The Common points are:"<<val<<endl;
    //   pcl::transformPointCloud(*outputpcl, *offsetcloud, transform2);
    //   viewer.updatePointCloud(offsetcloud,"original4_cloud");
    //   theta+= 1;
      
    // }
  
  }

  return (0);
}



// -------Calculating Normals for the read pointcloud--------
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

// --------------Another method to get the centroid----------
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

// ----------- Logic to find common points-------------
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