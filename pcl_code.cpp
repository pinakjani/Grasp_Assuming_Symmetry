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

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/pinak/PCL/test_pcd3.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout<<"Loaded "<<cloud->width*cloud->height<<"data points from test_pcd3.pcd"<< std::endl;
 
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cloud);
  Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
  Eigen::Vector3f major_vector, middle_vector, minor_vector; 
  pcl::PointCloud<pcl::PointXYZ>::Ptr orientedGolden(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Vector4f goldenMidPt = pca.getMean();
  Eigen::Matrix4f goldenTransform = Eigen::Matrix4f::Identity();
  goldenTransform.block<3, 3>(0, 0) = eigen_vectors;
  goldenTransform.block<4, 1>(0, 3) = goldenMidPt;
  pcl::transformPointCloud(*cloud, *orientedGolden, goldenTransform.inverse());
  std::cout<<goldenMidPt<<endl;
  // Eigen::Matrix3f rotation;
  // rotation << -1,0,0,
  //              0,1,0,
  //              0,0,-1;
//   pcl::PCA<pcl::PointXYZ> Tpca;
//   Tpca.setInputCloud(orientedGolden);
//   Eigen::Vector4f origin = Tpca.getMean();
//   cout<<"hey:"<<origin<<endl;
  // Eigen::Vector4f origin(0.03-5.03785e-07,-7.88388e-08,0.03-9.41368e-07,1);
  // Eigen::Vector3f origin(-0.0285931,-0.997756,0.0605409);

  // Eigen::Vector4f origin = goldenTransform.getMean();

  // // ------Initial Transfromation----------------
  // Eigen::Matrix4f pclTransform = Eigen::Matrix4f::Identity();
  // pclTransform.block<3, 3>(0, 0) = rotation;
  // pclTransform.block<4, 1>(0, 3) = origin;
  // pcl::transformPointCloud(*orientedGolden, *outputpcl, pclTransform.inverse());
  // ------------------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputpcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(orientedGolden);
  feature_extractor.compute();
  feature_extractor.getEigenVectors(major_vector,middle_vector,minor_vector);
  float theta = M_PI;
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  Eigen::Vector3f view(0,0,-1);
  Eigen::Vector3f axis;
 
  // cout<<"vector:"<<middle_vector<<major_vector<<minor_vector<<endl;
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
  // The same rotation matrix as before; theta radians around Z axis
  transform.rotate (Eigen::AngleAxisf (theta, axis));
  pcl::transformPointCloud(*orientedGolden, *outputpcl, transform);

  // Eigen::Vector4f cent;
  // pcl::compute3DCentroid(*orientedGolden,cent);
  // cout<<"dekho:"<<cent[0];
  // pcl::ModelCoefficients coeff;
  // coeff.values.push_back(cent[0]);
  // coeff.values.push_back(cent[1]);
  // coeff.values.push_back(cent[2]);
  // coeff.values.push_back(0.001);
  // cout<<"eigen_matrix"<<eigen_vectors<<endl;
//   cout<<"dekho1:"<<cent-origin<<endl;
   float max_y = INT_MIN;
   for (const auto& point: *orientedGolden){
     cout<<"x:"<<point.y<<endl;
     if(point.y>max_y)
      max_y = point.y;


   }

   float max_z = INT_MIN;
   float min_z = INT_MAX;
   for (const auto& pointa: *orientedGolden){
     cout<<"z:"<<pointa.z<<endl;
     if(pointa.z>max_z)
    { 
      max_z = pointa.z;
    }
    if(pointa.z<min_z){
        min_z = pointa.z;
   }
   }


   cout<<"MIN_Z:"<<min_z<<endl;

  float max_y2 = INT_MIN;
   for (const auto& pointn: *outputpcl){
    //  cout<<"z_n:"<<pointn.y<<endl;
     if(pointn.y>max_y2)
      max_y2 = pointn.y;

   }

   cout<<"MAX_X:"<<max_y<<" "<<max_y2;
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr offsetcloud(new pcl::PointCloud<pcl::PointXYZ>);
  //visualising the cloud
  pcl::visualization::PCLVisualizer viewer ("Simple pointcloud display example");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler3 (outputpcl, 20, 255, 20);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler2 (orientedGolden, 20, 20, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 255, 20, 25);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler4 (offsetcloud, 255, 100, 20);
  viewer.addPointCloud (outputpcl, source_cloud_color_handler3, "original3_cloud");
  viewer.addPointCloud (orientedGolden, source_cloud_color_handler2, "original2_cloud");
  viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");
  viewer.addPointCloud (offsetcloud, source_cloud_color_handler4, "original4_cloud");
  // viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals, 10, 0.05, "normals");
  // viewer.addCoordinateSystem (1.5, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  // viewer.addSphere(coeff);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original2_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original3_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original4_cloud");
  int count=0;
  float offset = 0;
  Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce (2);
    if (offset>-1*(max_z-min_z))
      offset = offset - 0.00001;
    else{
      cout<<"count:"<<count<<endl;
      // break;

  }
    transform2.translation()<< 0,-1*(max_y2-max_y),offset;
    cout<<"namaste"<<endl;
    int val = 0;
    theta = 0;
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
    
    pcl::transformPointCloud(*outputpcl, *offsetcloud, transform2);
    viewer.updatePointCloud(offsetcloud,"original4_cloud");
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
    
    cout<<"The Common points are:"<<val<<endl;
    cout<<"idhar dekho:"<<dot1<<" "<<dot2<<" "<<dot3<<endl;
    cout<<"MAX_X:"<<max_y<<" "<<max_y2<<" "<<(max_z-min_z)<<endl;
    count++;
    // cout<<"C1:"<<outputpcl->height<<" C2:"<<offsetcloud->height<<endl;
  }

  //visualising the normals of the cloud
  return (0);
}
