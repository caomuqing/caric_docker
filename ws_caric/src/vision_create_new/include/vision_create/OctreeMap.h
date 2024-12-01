#ifndef OCTREEMAP_H
#define OCTREEMAP_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_search.h>
#include <pcl/common/centroid.h>
class OctMap 
{
    public:
        OctMap();
        void Process(void);
        void Calculat_Center(void);
    public:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxel;
        pcl::PointCloud<pcl::PointXYZ>::VectorType voxelCentroids;
        double solution = 2.5;
};

#endif