


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>


int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "normal_method");

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    	ros::console::notifyLoggerLevelsChanged();

  	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    std::string pcd_name, bag_name, bag_topic;
    nh.param<std::string>("/pcd_to_bag/pcd_name", pcd_name, "/home/conor/ros_data/trees/Aerial Lidar/Raw_Data/a1.pcd");
    nh.param<std::string>("/pcd_to_bag/bag_name", bag_name, "/home/conor/ros_data/trees/Aerial Lidar/Bags/a1.bag");
    nh.param<std::string>("/pcd_to_bag/bag_topic", bag_topic, "lidar");
    ROS_INFO_STREAM("Loading input data from file " << pcd_name);

    ros::Time start_time = ros::Time::now();
    if(pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_name, *cloud) == -1)
    {
    	ROS_ERROR_STREAM("couldn't load pcd from file " << pcd_name);
    	return (-1);
    }
    ros::Duration load_time = ros::Time::now() - start_time;
    ROS_INFO_STREAM("loaded, in " << load_time << "sec! Size: " << cloud->points.size());

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = "map";

    rosbag::Bag bag;
    //std::string bag_name = "lidar_" + std::to_string(ros::Time::now().toSec()) + ".bag";
    bag.open(bag_name, rosbag::bagmode::Write);
    bag.write(bag_topic, ros::Time::now(), msg);
    ROS_INFO_STREAM("[LaserStitcher] Saved a ROSBAG to the file " << bag_name);

}