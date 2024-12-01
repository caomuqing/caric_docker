#include <ros/ros.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/Point.h>
#include "kios_solution/OctomapToCoords.h"  // Change to the name of your package
 
bool octomap_to_coords(kios_solution::OctomapToCoords::Request  &req,
                       kios_solution::OctomapToCoords::Response &res)
{
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(req.octomap);
    if (!tree){
        ROS_ERROR("Failed to create octree structure");
        return false;
    }
 
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
 
    for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it != end; ++it)
    {
        if(octree->isNodeOccupied(*it)){
            geometry_msgs::Point point;
            point.x = it.getX();
            point.y = it.getY();
            point.z = it.getZ();
            res.points.push_back(point);
        }
    }
 
    delete octree; // Don't forget to free memory
    return true;
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "octomap_to_coords_server");
    ros::NodeHandle n;
 
    ros::ServiceServer service = n.advertiseService("octomap_to_coords", octomap_to_coords);
    ROS_INFO("Ready to convert octomaps to coordinates.");
    ros::spin();
 
    return 0;
}
