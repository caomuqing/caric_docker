#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense> 
class PointCloudAccumulator
{
public:
    PointCloudAccumulator() : accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>)
    {
        pointcloud_subscriber = nh.subscribe("/jurong/cloud_inW", 50, &PointCloudAccumulator::pointcloudCallback, this);
        pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/Global_Map", 50);
    }

    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ> input_cloud;
        pcl::fromROSMsg(*cloud_msg, input_cloud);
        // Perform point cloud accumulation here
        
        *accumulated_cloud += input_cloud;
        //滤波

        //裁减掉box外的点云
        // Publish the accumulated point cloud
        sensor_msgs::PointCloud2 accumulated_cloud_msg;
        pcl::toROSMsg(*accumulated_cloud, accumulated_cloud_msg);
        accumulated_cloud_msg.header = cloud_msg->header;
        pointcloud_publisher.publish(accumulated_cloud_msg);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber pointcloud_subscriber;
    ros::Publisher pointcloud_publisher;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Map_Server");
    
    PointCloudAccumulator accumulator;
    ros::spin();
    return 0;
}