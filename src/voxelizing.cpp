#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int
main ()
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
  pcl::PCLPointCloud2::Ptr voxel_cloud (new pcl::PCLPointCloud2());
  pcl::PCDReader cloud_reader;
  pcl::PCDWriter cloud_writer;

  std::string path="/home/huy/dev_ws/src/my_package/Rtabmap/3rd/";
  // Reading the cloud
  cloud_reader.read (path+std::string("cloud.pcd"),*cloud);

  std::cout<<"Source Cloud Points "<< cloud->width * cloud->height<< std::endl;
  // Voxel
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(0.05,0.05,0.05);
  voxel_filter.filter(*voxel_cloud);


  std::cout<<"Voxel Cloud Points "<< voxel_cloud->width * voxel_cloud->height<< std::endl;

  // Write
  cloud_writer.write(path+std::string("voxelized_1.pcd"),*voxel_cloud,Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);


  return (0);
}