


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>


#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing.h>



int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "aerial_lidar");

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    	ros::console::notifyLoggerLevelsChanged();

  	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    // ----------------------------- Pull in data -----------------------------
    ros::Time start_time = ros::Time::now();
    bool load_pcd;
    nh.param<bool>("aerial_lidar/from_pcd", load_pcd, false);
    if(load_pcd)
    {
        std::string pcd_name;
        nh.param<std::string>("aerial_lidar/pcd_name", pcd_name, "/home/conor/ros_data/trees/Aerial Lidar/ascii_data.pcd");
        if(pcl::io::loadPCDFile<pcl::PointXYZI>("/home/conor/ros_data/trees/Aerial Lidar/ascii_data.pcd", *cloud) == -1)
        {
            ROS_ERROR_STREAM("couldn't load...");
            return (-1);
        }
    }
    else
    {
        std::string bag_name, bag_topic;
        nh.param<std::string>("aerial_lidar/bag_name", bag_name, "/home/conor/ros_data/trees/Aerial Lidar/Bags/a1.bag");
        nh.param<std::string>("aerial_lidar/bag_topic", bag_topic, "/lidar");
        ROS_INFO_STREAM("[Tester] Loading clouds from bag files, using bag name: " << bag_name << " and topic name: " << bag_topic << ".");
        sensor_msgs::PointCloud2 input_msg;
        // Open Bag
        rosbag::Bag input_bag; 
        input_bag.open(bag_name, rosbag::bagmode::Read);
        // Create Topic List
        std::vector<std::string> topics;
        topics.push_back(bag_topic);
        rosbag::View view(input_bag, rosbag::TopicQuery(topics));
        // Extract Cloud
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
            if (cloud_ptr != NULL)
                input_msg = *cloud_ptr;
            else
              ROS_ERROR_STREAM("[Tester] Cloud caught for first cloud is null...");
        }
        input_bag.close();
        ROS_INFO_STREAM("[Tester] Cloud size: " << input_msg.height*input_msg.width);
        pcl::fromROSMsg(input_msg, *cloud);
    }    
    ros::Duration load_time = ros::Time::now() - start_time;
    ROS_INFO_STREAM("Data loaded, in " << load_time << "sec! Size: " << cloud->points.size());

    bool load_second_bag;
    nh.param<bool>("aerial_lidar/load_second_bag", load_second_bag, false);
    if(load_second_bag)
    {
        std::string bag_name, bag_topic;
        nh.param<std::string>("aerial_lidar/bag_name_two", bag_name, "/home/conor/ros_data/trees/Aerial Lidar/Bags/a1.bag");
        nh.param<std::string>("aerial_lidar/bag_topic_two", bag_topic, "/lidar");
        ROS_INFO_STREAM("[Tester] Loading clouds from bag files, using bag name: " << bag_name << " and topic name: " << bag_topic << ".");
        sensor_msgs::PointCloud2 input_msg;
        // Open Bag
        rosbag::Bag input_bag; 
        input_bag.open(bag_name, rosbag::bagmode::Read);
        // Create Topic List
        std::vector<std::string> topics;
        topics.push_back(bag_topic);
        rosbag::View view(input_bag, rosbag::TopicQuery(topics));
        // Extract Cloud
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
            if (cloud_ptr != NULL)
                input_msg = *cloud_ptr;
            else
              ROS_ERROR_STREAM("[Tester] Cloud caught for first cloud is null...");
        }
        input_bag.close();
        ROS_INFO_STREAM("[Tester] Cloud size: " << input_msg.height*input_msg.width);

        sensor_msgs::PointCloud2 first_cloud, concatenated_clouds;
        pcl::toROSMsg(*cloud, first_cloud);
        pcl::concatenatePointCloud(first_cloud, input_msg, concatenated_clouds);
        pcl::fromROSMsg(concatenated_clouds, *cloud);

        ros::Duration load_time = ros::Time::now() - start_time;
        ROS_INFO_STREAM("Data loaded, in " << load_time << "sec! Size: " << cloud->points.size());
    }

    bool load_third_bag;
    nh.param<bool>("aerial_lidar/load_third_bag", load_third_bag, false);
    if(load_third_bag)
    {
        std::string bag_name, bag_topic;
        nh.param<std::string>("aerial_lidar/bag_name_three", bag_name, "/home/conor/ros_data/trees/Aerial Lidar/Bags/a1.bag");
        nh.param<std::string>("aerial_lidar/bag_topic_three", bag_topic, "/lidar");
        ROS_INFO_STREAM("[Tester] Loading clouds from bag files, using bag name: " << bag_name << " and topic name: " << bag_topic << ".");
        sensor_msgs::PointCloud2 input_msg;
        // Open Bag
        rosbag::Bag input_bag; 
        input_bag.open(bag_name, rosbag::bagmode::Read);
        // Create Topic List
        std::vector<std::string> topics;
        topics.push_back(bag_topic);
        rosbag::View view(input_bag, rosbag::TopicQuery(topics));
        // Extract Cloud
        BOOST_FOREACH(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
            if (cloud_ptr != NULL)
                input_msg = *cloud_ptr;
            else
              ROS_ERROR_STREAM("[Tester] Cloud caught for first cloud is null...");
        }
        input_bag.close();
        ROS_INFO_STREAM("[Tester] Cloud size: " << input_msg.height*input_msg.width);

        sensor_msgs::PointCloud2 first_cloud, concatenated_clouds;
        pcl::toROSMsg(*cloud, first_cloud);
        pcl::concatenatePointCloud(first_cloud, input_msg, concatenated_clouds);
        pcl::fromROSMsg(concatenated_clouds, *cloud);

        ros::Duration load_time = ros::Time::now() - start_time;
        ROS_INFO_STREAM("Data loaded, in " << load_time << "sec! Size: " << cloud->points.size());
    }


    float northing_center;
    float easting_center;
    float northing_radius;  // in meters, approximately 
    float easting_radius;   // in meters, approximately 

    nh.param<float>("aerial_lidar/northing_center", northing_center, 3350841.3);
    nh.param<float>("aerial_lidar/easting_center", easting_center, 621735.5);
    nh.param<float>("aerial_lidar/northing_radius", northing_radius, 100);
    nh.param<float>("aerial_lidar/easting_radius", easting_radius, 100);



    // Remove points obviously too far away from center of cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr outlierless_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for(int i=0; i<cloud->points.size(); i++)
        if( fabs(cloud->points[i].x - easting_center) < easting_radius*1.3 )
            if( fabs(cloud->points[i].y - northing_center) < northing_radius*1.3 )
                outlierless_cloud->points.push_back(cloud->points[i]);


    // Subset cloud by target region
    ROS_INFO_STREAM("Subsetting cloud using center at " << easting_center << "x" << northing_center << " and radii " << easting_radius << "x" << northing_radius);
    // Initialize cloud, means
    float min_easting = 10e20;
    float min_northing = 10e20;
    float max_easting = -10e20;
    float max_northing = -10e20;
    pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_cloud(new pcl::PointCloud<pcl::PointXYZI>());    
    // Perform subsetting
    for(int i=0; i<outlierless_cloud->points.size(); i++)
    {
        // Check for new min/max
        if(outlierless_cloud->points[i].x < min_easting) 
            min_easting = outlierless_cloud->points[i].x;
        if(outlierless_cloud->points[i].y < min_northing) 
            min_northing = outlierless_cloud->points[i].y;
        if(outlierless_cloud->points[i].x > max_easting) 
            max_easting = outlierless_cloud->points[i].x;
        if(outlierless_cloud->points[i].y > max_northing) 
            max_northing = outlierless_cloud->points[i].y;

        // Center full cloud on target point
        outlierless_cloud->points[i].x -= easting_center;
        outlierless_cloud->points[i].y -= northing_center;
        // Extract points within target region 
        if( fabs(outlierless_cloud->points[i].x) < easting_radius )
            if( fabs(outlierless_cloud->points[i].y) < northing_radius )
                clipped_cloud->points.push_back(outlierless_cloud->points[i]);
    }
    float easting_midpoint = (max_easting - min_easting)/2 + min_easting;
    float northing_midpoint = (max_northing - min_northing)/2 + min_northing;
    ROS_INFO_STREAM("Subset cloud. Outlierless size: " << outlierless_cloud->points.size() << " Final size: " << clipped_cloud->points.size());
    ROS_INFO_STREAM("  X Limits: " << min_easting << " - " << max_easting << "; Y Limits: " << min_northing << " - " << max_northing << "; Midpoints: " << easting_midpoint << " " << northing_midpoint);
    float point_density = float(outlierless_cloud->points.size()) / (max_easting - min_easting) / (max_northing - min_northing);
    ROS_INFO_STREAM("  Overall density: " << point_density);

    float min_easting_subset = 10e20;
    float min_northing_subset = 10e20;
    float max_easting_subset = -10e20;
    float max_northing_subset = -10e20;
    // Perform subsetting
    for(int i=0; i<clipped_cloud->points.size(); i++)
    {
        pcl::PointXYZI point = clipped_cloud->points[i];
        // Check for new min/max
        if(point.x < min_easting_subset) 
            min_easting_subset = point.x;
        if(point.y < min_northing_subset) 
            min_northing_subset = point.y; 
        if(point.x > max_easting_subset) 
            max_easting_subset = point.x;
        if(point.y > max_northing_subset) 
            max_northing_subset = point.y;
    }
    float easting_midpoint_subset = (max_easting_subset - min_easting_subset)/2 + min_easting_subset;
    float northing_midpoint_subset = (max_northing_subset - min_northing_subset)/2 + min_northing_subset;
    ROS_INFO_STREAM("  Subset! X Limits: " << min_easting_subset << " - " << max_easting_subset << "; Y Limits: " << min_northing_subset << " - " << max_northing_subset << "; Midpoints: " << easting_midpoint_subset << " " << northing_midpoint_subset);
    float point_density_subset = float(clipped_cloud->points.size()) / (max_easting_subset - min_easting_subset) / (max_northing_subset - min_northing_subset);
    ROS_INFO_STREAM("  Subset density: " << point_density_subset);

    // Set outputs in ROS Msg format
    sensor_msgs::PointCloud2 subset_msg;
    pcl::toROSMsg(*clipped_cloud, subset_msg);
    subset_msg.header.frame_id = "map";
    // Set outputs in ROS Msg format
    sensor_msgs::PointCloud2 total_msg; 
    pcl::toROSMsg(*outlierless_cloud, total_msg);
    total_msg.header.frame_id = "map";

    // Publishers
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("subset_lidar", 1);
    ros::Publisher pub_total = nh.advertise<sensor_msgs::PointCloud2>("total_lidar", 1);
    // Save
    rosbag::Bag bag;
    std::string bag_name = "lidar_scene" + std::to_string(ros::Time::now().toSec()) + ".bag";
    bag.open(bag_name, rosbag::bagmode::Write);
    bag.write("subset_lidar", ros::Time::now(), subset_msg);
    ROS_INFO_STREAM("[LaserStitcher] Saved a ROSBAG to the file " << bag_name);

/*  save multiple clouds concatenated
    rosbag::Bag bag2;
    std::string bag_name2 = "total_scene" + std::to_string(ros::Time::now().toSec()) + ".bag";
    bag2.open(bag_name2, rosbag::bagmode::Write);
    bag2.write("subset_lidar", ros::Time::now(), total_msg);
    ROS_INFO_STREAM("[LaserStitcher] Saved a ROSBAG to the file " << bag_name2);
*/

/*
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr minima_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(voxelized_cloud);

    for(int i=0; i<normal_cloud->points.size(); i++)
    {
        pcl::PointXYZI point;
        point.x = normal_cloud->points[i].x;
        point.y = normal_cloud->points[i].y;
        point.z = normal_cloud->points[i].z;

        std::vector<int> nearest_indices;
        std::vector<float> nearest_dist_squareds;

        pcl::PointXYZINormal point_n;
        point_n.x = normal_cloud->points[i].x;
        point_n.y = normal_cloud->points[i].y;
        point_n.z = normal_cloud->points[i].z;
        point_n.normal_x = 0;
        point_n.normal_y = 0;
        point_n.normal_z = 0;

        pcl::PointXYZINormal source_point = normal_cloud->points[i];
        if ( kdtree.nearestKSearch (point, 15, nearest_indices, nearest_dist_squareds) > 0 )
*/



    // Output
    while(ros::ok())
    {
    	pub.publish(subset_msg);
        pub_total.publish(total_msg);
    	ros::Duration(1.0).sleep();
    }

}