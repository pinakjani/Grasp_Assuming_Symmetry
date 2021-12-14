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
int i =0;
std::string path= "/home/pinak/PCL/PCD/test";
  // Reading stored pointcloud of pointXYZ in cloud variable
    pcl::PointCloud<pcl::PointXYZ>::Ptr ref (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr offsetcloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/pinak/PCL/testref.pcd", *ref) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test3.pcd \n");
        return (-1);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/pinak/PCL/sample.pcd", *cloud) == -1) //* load the file
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
        if(i<2600){
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (path+std::to_string(i)+".pcd", *offsetcloud) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read file test1.pcd \n");
                return (-1);
            }
            cout<<"PCD"+std::to_string(i)<<endl;
            viewer.updatePointCloud(offsetcloud,"original4_cloud");
            i++;
        }

 }
return (0);
}
