#include "OctreeMap.h"
#include <ros/ros.h>
#include <thread>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <visualization_msgs/Marker.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <string>
#include <chrono>
#include <random>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include "Pcl_Pro.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree (3.0f);
void OctMap::Calculat_Center(void) 
{
    //for (pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::FixedDepthIterator tree_it = octree.fixed_depth_begin (depth);
    //     tree_it != octree.fixed_depth_end ();
    //     ++tree_it)
    //{
       // Compute the point at the center of the voxel which represents the current OctreeNode
      //Eigen::Vector3f voxel_min, voxel_max;
     // octree.getVoxelBounds (tree_it, voxel_min, voxel_max);

     // pt_voxel_center.x = (voxel_min.x () + voxel_max.x ()) / 2.0f;
      //pt_voxel_center.y = (voxel_min.y () + voxel_max.y ()) / 2.0f;
      //pt_voxel_center.z = (voxel_min.z () + voxel_max.z ()) / 2.0f;
     // cloudVoxel->points.push_back (pt_voxel_center);
    //}
}
void OctMap::Process(void) 
{
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    octree.getVoxelCentroids (voxelCentroids);
    // std::cout << "叶子数量:" << voxelCentroids.size() <<std::endl;
    // std::cout << "centerid:x" << voxelCentroids[0].x <<"y:"<<voxelCentroids[0].y <<"z:"<<voxelCentroids[0].z <<std::endl;
}
//create
OctMap::OctMap() 
{   
    cloudVoxel = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}