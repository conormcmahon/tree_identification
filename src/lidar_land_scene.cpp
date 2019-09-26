

#include "tree_identification/lidar_land_scene.h"

// Explicitly Declare Some Templated Versions
template class LiDARLandScene<pcl::PointXYZI, pcl::PointXYZINormal, pcl::VegetationPoint3>;


template <typename PointType, typename PointNormalType, typename VegetationPointType>
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::LiDARLandScene():
	input_cloud_(new PCN()),
	vegetation_cloud_(new VC()),
	publishers_initialized_(false)
{
	scene_transform_.header.frame_id = "map";
	scene_transform_.child_frame_id = "lidar_scene_frame" + std::to_string(ros::Time::now().toSec());
	scene_transform_.transform.translation.x = 0;
	scene_transform_.transform.translation.y = 0;
	scene_transform_.transform.translation.z = 0;
	scene_transform_.transform.rotation.x = 0;
	scene_transform_.transform.rotation.y = 0;
	scene_transform_.transform.rotation.z = 0;
	scene_transform_.transform.rotation.w = 1;
}


template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::inputROSBAG(std::string bag_name, std::string bag_topic)
{
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
            input_msg_ = *cloud_ptr;
        else
          ROS_ERROR_STREAM("[LiDARLandScene] Cloud caught for first cloud is null...");
    }
    input_bag.close();
    ROS_INFO_STREAM("[LiDARLandScene] Input cloud size: " << input_msg_.height*input_msg_.width);

    pcl::fromROSMsg(input_msg_, *input_cloud_);
}


template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::loadLiDARScene(std::string parent_directory_path)
{
	// Load Input Data
	loadCloudBag(parent_directory_path + "input/cloud.bag", "input_cloud", input_msg_);
	pcl::fromROSMsg(input_msg_, *input_cloud_);
	// Load DEM Data
	loadCloudBag(parent_directory_path + "dem/cloud.bag", "dem_cloud", dem_image_.cloud_msg);
	pcl::fromROSMsg(dem_image_.cloud_msg, *dem_image_.cloud);
	loadImageBag(parent_directory_path + "dem/image.bag", "dem_image", dem_image_.image_msg);
	dem_image_.image = cv_bridge::toCvCopy(dem_image_.image_msg, sensor_msgs::image_encodings::MONO8);
	// Load DSM Data
	loadCloudBag(parent_directory_path + "dsm/cloud.bag", "dsm_cloud", dsm_image_.cloud_msg);
	pcl::fromROSMsg(dsm_image_.cloud_msg, *dsm_image_.cloud);
	loadImageBag(parent_directory_path + "dsm/image.bag", "dsm_image", dsm_image_.image_msg);
	dsm_image_.image = cv_bridge::toCvCopy(dsm_image_.image_msg, sensor_msgs::image_encodings::MONO8);
	// Load CHM Data
	loadCloudBag(parent_directory_path + "chm/cloud.bag", "chm_cloud", chm_image_.cloud_msg);
	pcl::fromROSMsg(chm_image_.cloud_msg, *chm_image_.cloud);
	loadImageBag(parent_directory_path + "chm/image.bag", "chm_image", chm_image_.image_msg);
	chm_image_.image = cv_bridge::toCvCopy(chm_image_.image_msg, sensor_msgs::image_encodings::MONO8);
	// Load Vegetation Data
	loadCloudBag(parent_directory_path + "vegetation/cloud.bag", "vegetation_cloud", vegetation_msg_);
	pcl::fromROSMsg(vegetation_msg_, *vegetation_cloud_);
	// Load Spatial Transform Data
	loadTransformBag(parent_directory_path + "transform/transform.bag", "transform", scene_transform_);
}
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::loadCloudBag(std::string bag_path, std::string bag_topic, sensor_msgs::PointCloud2 &msg)
{
	// Open Bag
    rosbag::Bag input_bag; 
    input_bag.open(bag_path, rosbag::bagmode::Read);
    // Create Topic List
    std::vector<std::string> topics;
    topics.push_back(bag_topic);
    rosbag::View view(input_bag, rosbag::TopicQuery(topics));
    // Extract Cloud
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::PointCloud2::ConstPtr cloud_ptr = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_ptr != NULL)
            msg = *cloud_ptr;
        else
          ROS_ERROR_STREAM("[LiDARLandScene] Cloud caught for first cloud is null...");
    }
    input_bag.close();
    ROS_INFO_STREAM("[LiDARLandScene] Loaded cloud size: " << msg.height*msg.width << " from bag " << bag_path << " using topic " << bag_topic);
}
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::loadImageBag(std::string bag_path, std::string bag_topic, sensor_msgs::Image &msg)
{
	// Open Bag
    rosbag::Bag input_bag; 
    input_bag.open(bag_path, rosbag::bagmode::Read);
    // Create Topic List
    std::vector<std::string> topics;
    topics.push_back(bag_topic);
    rosbag::View view(input_bag, rosbag::TopicQuery(topics));
    // Extract Cloud
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::Image::ConstPtr image_ptr = m.instantiate<sensor_msgs::Image>();
        if (image_ptr != NULL)
            msg = *image_ptr;
        else
          ROS_ERROR_STREAM("[LiDARLandScene] Cloud caught for first cloud is null...");
    }
    input_bag.close();
    ROS_INFO_STREAM("[LiDARLandScene] Loaded image size: " << msg.height*msg.width << " from bag " << bag_path << " using topic " << bag_topic);
}
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::loadTransformBag(std::string bag_path, std::string bag_topic, geometry_msgs::TransformStamped &msg)
{
	// Open Bag
    rosbag::Bag input_bag; 
    input_bag.open(bag_path, rosbag::bagmode::Read);
    // Create Topic List
    std::vector<std::string> topics;
    topics.push_back(bag_topic);
    rosbag::View view(input_bag, rosbag::TopicQuery(topics));
    // Extract Cloud
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        geometry_msgs::TransformStamped::ConstPtr transform_ptr = m.instantiate<geometry_msgs::TransformStamped>();
        if (transform_ptr != NULL)
            msg = *transform_ptr;
        else
          ROS_ERROR_STREAM("[LiDARLandScene] Cloud caught for first cloud is null...");
    }
    input_bag.close();
    ROS_INFO_STREAM("[LiDARLandScene] Loaded transform: " << msg << " from bag " << bag_path << " using topic " << bag_topic);
}


template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::saveLandScene(std::string parent_directory_path)
{
	// Return if input path is empty
	if(parent_directory_path.length() < 1)
	{
		ROS_ERROR_STREAM("[LiDARLandScene] LandScene saving operation requested, but input bag path is empty: " << parent_directory_path);
		return;
	}
	// Save Input Cloud
	savePointCloud(parent_directory_path + "input/", "input_cloud", input_msg_);
	// Save DEM Cloud
	savePointCloud(parent_directory_path + "dem/", "dem_cloud", dem_image_.cloud_msg);
	saveImage(parent_directory_path + "dem/", "dem_image", dem_image_.image_msg, dem_image_.image);
	// Save DSM Cloud
	savePointCloud(parent_directory_path + "dsm/", "dsm_cloud", dsm_image_.cloud_msg);
	saveImage(parent_directory_path + "dsm/", "dsm_image", dsm_image_.image_msg, dsm_image_.image);
	// Save CHM Cloud
	savePointCloud(parent_directory_path + "chm/", "chm_cloud", chm_image_.cloud_msg);
	saveImage(parent_directory_path + "chm/", "chm_image", chm_image_.image_msg, chm_image_.image);
	// Save DEM Cloud
	savePointCloud(parent_directory_path + "vegetation/", "vegetation_cloud", vegetation_msg_);
	// Save Scene Transform
	saveTransform(parent_directory_path + "transform/", "transform", scene_transform_);
}
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::savePointCloud(std::string bag_path, std::string bag_topic, sensor_msgs::PointCloud2 cloud_msg)
{
	// Create the target directory if it does not exist
	createDirectory(bag_path);
	rosbag::Bag bag;
	bag.open(bag_path + "cloud.bag", rosbag::bagmode::Write);
	bag.write(bag_topic, ros::Time::now(), cloud_msg);
	ROS_INFO_STREAM("[LiDARLandScene] Saved a ROSBAG to the file " << bag_path + "cloud.bag");
}
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::saveImage(std::string bag_path, std::string bag_topic, sensor_msgs::Image image_msg, cv_bridge::CvImagePtr image)
{
	createDirectory(bag_path);
	rosbag::Bag bag;
	bag.open(bag_path + "image.bag", rosbag::bagmode::Write);
	bag.write(bag_topic, ros::Time::now(), image_msg);
	ROS_INFO_STREAM("[LiDARLandScene] Saved a ROSBAG to the file " << bag_path + "image.bag");
	cv::imwrite(bag_path + "image.jpg", image->image);
}
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::saveTransform(std::string bag_path, std::string bag_topic, geometry_msgs::TransformStamped scene_transform)
{
	createDirectory(bag_path);
	rosbag::Bag bag;
	bag.open(bag_path + "transform.bag", rosbag::bagmode::Write);
	bag.write(bag_topic, ros::Time::now(), scene_transform);
	ROS_INFO_STREAM("[LiDARLandScene] Saved a ROSBAG to the file " << bag_path + "transform.bag");
}
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::createDirectory(std::string bag_path)
{			
	boost::filesystem::path boost_path(bag_path);
	if(!boost::filesystem::exists(boost_path))
	{
		ROS_INFO_STREAM("[LiDARLandScene] In order to save LiDARLandScene, creating new directory at path " << bag_path);
		boost::filesystem::create_directories(boost_path);
		return;
	}
	else
		if(!boost::filesystem::is_directory(boost_path))
			ROS_ERROR_STREAM("[LiDARLandScene] During saving process, path " << bag_path << " already exists, but is not a directory.");
}

template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::setFrameNames(std::string parent_frame, std::string scene_frame)
{
	scene_transform_.header.frame_id = parent_frame;
	scene_transform_.child_frame_id = scene_frame;
}


template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::setPublishingNamespace(std::string publishing_namespace, ros::NodeHandle nh)
{
	// Initialize Publishers
	publishers_initialized_ = true;
	input_pub_ = nh.advertise<sensor_msgs::PointCloud2>(publishing_namespace + "/input_cloud", 1);
	dem_image_.cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(publishing_namespace + "/dem_cloud", 1);
	dem_image_.image_pub = nh.advertise<sensor_msgs::Image>(publishing_namespace + "/dem_image", 1);
	dem_filtered_image_.cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(publishing_namespace + "/dem_filtered_cloud", 1);
	dem_filtered_image_.image_pub = nh.advertise<sensor_msgs::Image>(publishing_namespace + "/dem_filtered_image", 1);
	dsm_image_.cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(publishing_namespace + "/dsm_cloud", 1);
	dsm_image_.image_pub = nh.advertise<sensor_msgs::Image>(publishing_namespace + "/dsm_image", 1);
	chm_image_.cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(publishing_namespace + "/chm_cloud", 1);
	chm_image_.image_pub = nh.advertise<sensor_msgs::Image>(publishing_namespace + "/chm_image", 1);
	vegetation_pub_ = nh.advertise<sensor_msgs::PointCloud2>(publishing_namespace + "/vegetation_cloud", 1);
}


template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::evaluateScene()
{
	findSceneLimits(true);
	inputNormals();
	splitToPixels();
	simpleDEM();
	clusterFilterDEM();
	waterEstimation();
	simpleDSM();
	simpleCHM();
	vegetationCloud();
}

// -------- Get Scene Limits --------
// Determine limits in northing/easting of scene and image size
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::setPixelSize(float pixel_size)
{
	pixel_size_ = pixel_size;
}


// -------- Get Scene Limits --------
// Determine limits in northing/easting of scene and image size
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::findSceneLimits(bool realign)
{
	ROS_DEBUG_STREAM("[LiDARLandScene] Determining Scene Spatial Limits:");
	// Find range limits of the cloud
	min_east_ = 10e10;
	max_east_ = -10e10;
	min_north_ = 10e10;
	max_north_ = -10e10;
	for(int i=0; i<input_cloud_->points.size(); i++)
	{
		PointNormalType point = input_cloud_->points[i];
        if(point.x < min_east_) 
            min_east_ = point.x;
        if(point.x > max_east_) 
            max_east_ = point.x;
        if(point.y < min_north_) 
            min_north_ = point.y;
        if(point.y > max_north_) 
            max_north_ = point.y;
	}
	ROS_DEBUG_STREAM("[LiDARLandScene]   Determined image limits. Easting: " << min_east_ << " to " << max_east_ << "; Northing: " << min_north_ << " to " << max_north_);
	if(realign)
	{
		// Realign scene to have its minima at the origin 
		// Take up spatial slack in the scene_transform_ object
		max_east_ -= min_east_;
		max_north_ -= min_north_;	
		scene_transform_.transform.translation.x = min_east_;
		scene_transform_.transform.translation.y = min_north_;
		for(int i=0; i<input_cloud_->points.size(); i++)
		{
			input_cloud_->points[i].x -= min_east_;
			input_cloud_->points[i].y -= min_north_;
		}
		min_east_ = 0;
		min_north_ = 0;
		ROS_DEBUG_STREAM("[LiDARLandScene]   Tared image limits. Easting: " << min_east_ << " to " << max_east_ << "; Northing: " << min_north_ << " to " << max_north_);
	}
	// Image width in pixels
	image_width_ = ceil((max_east_ - min_east_) / pixel_size_); 		// size in pixels
	image_height_ = ceil((max_north_ - min_north_) / pixel_size_); 	// size in pixels
	ROS_DEBUG_STREAM("[LiDARLandScene]   Determined image size: " << image_width_ << " by " << image_height_ << " pixels.");
}


// -------- Set Image Height Limits --------
// Set height limits for color scale in output images
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::setImageHeightLimits(float dem_min_height, float dem_max_height, float dem_filtered_min_height, float dem_filtered_max_height, float dsm_min_height, float dsm_max_height, float chm_min_height, float chm_max_height)
{
	dem_image_.raster_height_min = dem_min_height;
	dem_image_.raster_height_max = dem_max_height;
	dem_filtered_image_.raster_height_min = dem_min_height;
	dem_filtered_image_.raster_height_max = dem_max_height;
	dsm_image_.raster_height_min = dsm_min_height;
	dsm_image_.raster_height_max = dsm_max_height;
	chm_image_.raster_height_min = chm_min_height;
	chm_image_.raster_height_max = chm_max_height;
}


// -------- Set Normal Threshold --------
// Set height limits for color scale in output images
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::setNormalThreshold(float normal_threshold)
{
	normal_threshold_ = normal_threshold;
}


// -------- Set Cluster Tolerance --------
// Set maximum distance a DEM point can be from the rest of the ground cluster and still be included *UTM units) 
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::setClusterTolerance(float cluster_tolerance)
{
	cluster_tolerance_ = cluster_tolerance;
}


// -------- Generate Msgs --------
// Generate Output Messages in ROS format
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::generateMsgs()
{
	// Generate Point Cloud Messages
	pcl::toROSMsg(*input_cloud_, input_msg_);
	pcl::toROSMsg(*dem_image_.cloud, dem_image_.cloud_msg);
	pcl::toROSMsg(*dem_filtered_image_.cloud, dem_filtered_image_.cloud_msg);
	pcl::toROSMsg(*dsm_image_.cloud, dsm_image_.cloud_msg);
	pcl::toROSMsg(*chm_image_.cloud, chm_image_.cloud_msg);
	pcl::toROSMsg(*vegetation_cloud_, vegetation_msg_);
	input_msg_.header.frame_id = scene_transform_.child_frame_id;
	dem_image_.cloud_msg.header.frame_id = scene_transform_.child_frame_id;
	dem_filtered_image_.cloud_msg.header.frame_id = scene_transform_.child_frame_id;
	dsm_image_.cloud_msg.header.frame_id = scene_transform_.child_frame_id;
	chm_image_.cloud_msg.header.frame_id = scene_transform_.child_frame_id;
	vegetation_msg_.header.frame_id = scene_transform_.child_frame_id;
	// Generate Raster Image Messages
	dem_image_.image_msg = *dem_image_.image->toImageMsg();
	dsm_image_.image_msg = *dsm_image_.image->toImageMsg();
	chm_image_.image_msg = *chm_image_.image->toImageMsg();
	dem_image_.image_msg.encoding = sensor_msgs::image_encodings::MONO8;
	dsm_image_.image_msg.encoding = sensor_msgs::image_encodings::MONO8;
	chm_image_.image_msg.encoding = sensor_msgs::image_encodings::MONO8;
}



// -------- Get Clouds --------
// Returns copies of the internally stored PCL clouds  
template <typename PointType, typename PointNormalType, typename VegetationPointType> pcl::PointCloud<PointNormalType>
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::getInputCloud() { return *input_cloud_; }
template <typename PointType, typename PointNormalType, typename VegetationPointType> pcl::PointCloud<PointNormalType>
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::getDEMCloud() { return *dem_image_.cloud; }
template <typename PointType, typename PointNormalType, typename VegetationPointType> pcl::PointCloud<PointNormalType>
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::getDEMFilteredCloud() { return *dem_filtered_image_.cloud; }
template <typename PointType, typename PointNormalType, typename VegetationPointType> pcl::PointCloud<PointNormalType>
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::getDSMCloud() { return *dsm_image_.cloud; }
template <typename PointType, typename PointNormalType, typename VegetationPointType> pcl::PointCloud<PointNormalType>
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::getCHMCloud() { return *chm_image_.cloud; }
template <typename PointType, typename PointNormalType, typename VegetationPointType> pcl::PointCloud<VegetationPointType>
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::getVegetationCloud() { return *vegetation_cloud_; }  
// -------- Get Images --------
// Returns copies of the internally stored OpenCV images
template <typename PointType, typename PointNormalType, typename VegetationPointType> cv_bridge::CvImage
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::getDEMImage() { return *dem_image_.image; }
template <typename PointType, typename PointNormalType, typename VegetationPointType> cv_bridge::CvImage
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::getDSMImage() { return *dsm_image_.image; }
template <typename PointType, typename PointNormalType, typename VegetationPointType> cv_bridge::CvImage
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::getCHMImage() { return *chm_image_.image; }



// -------- Get Cloud Msgs --------
// Returns the output messages in ROS format
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::getRasterCloudMsgs(sensor_msgs::PointCloud2 &dem_msg, sensor_msgs::PointCloud2 &dsm_msg, sensor_msgs::PointCloud2 &chm_msg)
{ 
	dem_msg = dem_image_.cloud_msg;
	dsm_msg = dsm_image_.cloud_msg;
	chm_msg = chm_image_.cloud_msg;
}
// Returns the output messages in ROS format
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::getVegetationCloudMsg(sensor_msgs::PointCloud2 &vegetation_msg)
{ 
	vegetation_msg = vegetation_msg_;
}

// -------- Set Image Height Limits --------
// Set height limits for color scale in output images
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::getImageMsgs(sensor_msgs::Image &dem_image, sensor_msgs::Image &dsm_image, sensor_msgs::Image &chm_image)
{
	dem_image = dem_image_.image_msg;
	dsm_image = dsm_image_.image_msg;
	chm_image = chm_image_.image_msg;
}

// -------- Estimate Image Height Limits --------
// Base limits off min/max points in input cloud
template <typename PointType, typename PointNormalType, typename VegetationPointType> geometry_msgs::TransformStamped
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::getTransform()
{
	return scene_transform_;
}

// -------- Get Image Height --------
// Return image height in pixels
template <typename PointType, typename PointNormalType, typename VegetationPointType> float
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::getImageHeight()
{
	return image_height_;
}
// -------- Get Image Width --------
// Return image height in pixels
template <typename PointType, typename PointNormalType, typename VegetationPointType> float
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::getImageWidth()
{
	return image_width_;
}
// -------- Get Pixel Size --------
// Return size of each pixel in UTM units
template <typename PointType, typename PointNormalType, typename VegetationPointType> float
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::getPixelSize()
{
	return pixel_size_;
}

// -------- Estimate Image Height Limits --------
// Base limits off min/max points in input cloud
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::estimateImageHeightLimits()
{

}	


// -------- Input Normals --------
// Evaluate local normals over entire input cloud (expensive)
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::inputNormals()
{
	ROS_INFO_STREAM("[LiDARLandScene] Estimating normals for input cloud.");
	// Populate DEM Normals
	pcl::NormalEstimation<PointType, PointNormalType> norm_est;
	PCP temp_cloud(new PC());
	for(int i=0; i<input_cloud_->points.size(); i++)
	{
		PointType point;
		point.x = input_cloud_->points[i].x;
		point.y = input_cloud_->points[i].y;
		point.z = input_cloud_->points[i].z;
		point.intensity = input_cloud_->points[i].intensity;
		temp_cloud->points.push_back(point);
	}
	KDP normals_tree (new KD ());
	norm_est.setSearchMethod (normals_tree);
	norm_est.setKSearch (15);
	norm_est.setInputCloud (temp_cloud);
	norm_est.setViewPoint(0.0, 0.0, 1000);
	norm_est.compute(*input_cloud_);
	ROS_INFO_STREAM("[LiDARLandScene]   Finished estimating normals for input cloud.");
}



// -------- Subset LiDAR Scene --------
// Subset an existing LiDARLandScene object to a smaller spatial extent
//   "center" inputs are the center point of the new map, in UTM units
//   "radius" is radius of circular subset region around center, in UTM units
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::subsetLiDARScene(LiDARLandScene *input_scene, float northing_center, float easting_center, float radius)
{
	// Import Basic Settings
	pixel_size_ = input_scene->getPixelSize();
	float easting_transform = input_scene->getTransform().transform.translation.x;
	float northing_transform = input_scene->getTransform().transform.translation.y;
	scene_transform_.transform.translation.x = easting_transform;
	scene_transform_.transform.translation.y = northing_transform;

	ROS_INFO_STREAM("[LiDARLandScene] Subsetting an input land scene. Center coordinates: " << northing_center << " x " << easting_center << ", with radius " << radius << ". Input cloud's transform - Northing: " << northing_transform << " Easting: " << easting_transform);
	subsetPointCloud(input_scene->getInputCloud(), input_cloud_, northing_center, easting_center, radius, northing_transform, easting_transform);
	findSceneLimits(false);
	subsetPointCloud(input_scene->getDEMCloud(), dem_image_.cloud, northing_center, easting_center, radius, northing_transform, easting_transform);
	rasterizeCloud(dem_image_);
	subsetPointCloud(input_scene->getDEMFilteredCloud(), dem_filtered_image_.cloud, northing_center, easting_center, radius, northing_transform, easting_transform);
	rasterizeCloud(dem_filtered_image_);
	subsetPointCloud(input_scene->getDSMCloud(), dsm_image_.cloud, northing_center, easting_center, radius, northing_transform, easting_transform);
	rasterizeCloud(dsm_image_);
	subsetPointCloud(input_scene->getCHMCloud(), chm_image_.cloud, northing_center, easting_center, radius, northing_transform, easting_transform);
	rasterizeCloud(chm_image_);
	subsetPointCloud(input_scene->getVegetationCloud(), vegetation_cloud_, northing_center, easting_center, radius, northing_transform, easting_transform);
	// need to also put code to subset raster images
}

// -------- Subset Point Cloud --------
// Spatially subset an existing Point Cloud 
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::subsetPointCloud(PCN input_cloud, PCNP output_cloud, float northing_center, float easting_center, float radius, float northing_transform, float easting_transform)
{
	for(int i=0; i<input_cloud.points.size(); i++)
	{
		float easting_offset = input_cloud.points[i].x + easting_transform - easting_center;
		float northing_offset = input_cloud.points[i].y + northing_transform - northing_center;
		float distance = sqrt(pow(easting_offset,2) + pow(northing_offset,2));
		if(distance < radius)
			output_cloud->points.push_back(input_cloud.points[i]);
	}
	ROS_DEBUG_STREAM("[LiDARLandScene] Subset a point cloud. Initial size: " << input_cloud.points.size() << " Final size: " << output_cloud->points.size());
}
// -------- Subset Point Cloud --------
// Spatially subset an existing Point Cloud 
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::subsetPointCloud(VC input_cloud, VCP output_cloud, float northing_center, float easting_center, float radius, float northing_transform, float easting_transform)
{
	for(int i=0; i<input_cloud.points.size(); i++)
	{
		float easting_offset = input_cloud.points[i].x + easting_transform - easting_center;
		float northing_offset = input_cloud.points[i].y + northing_transform - northing_center;
		float distance = sqrt(pow(easting_offset,2) + pow(northing_offset,2));
		if(distance < radius)
			output_cloud->points.push_back(input_cloud.points[i]);
	}
	ROS_DEBUG_STREAM("[LiDARLandScene] Subset a point cloud. Initial size: " << input_cloud.points.size() << " Final size: " << output_cloud->points.size());
}


// -------- Split To Pixels --------
// Create list of point indices within each pixel
//template typename<PointType>
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::splitToPixels()
{
	point_raster_list_.resize(image_width_, std::vector< std::vector<int> >(image_height_, std::vector<int>(0)));
	// Generate gridded list of point indices from the cloud within each raster pixel
	for(int i=0; i<input_cloud_->points.size(); i++)
	{
		// (i,j) Position of point within the raster grid
		int column 	= floor( (input_cloud_->points[i].x - min_east_) / pixel_size_ );
		int row 	= floor( (input_cloud_->points[i].y - min_north_) / pixel_size_ );
		// Add the point's index to the raster grid
		point_raster_list_[column][row].push_back(i);
	}	
	ROS_INFO_STREAM("[LiDARLandScene] Split points to pixels.");
}

// -------- Simple DEM --------
// Estimate ground points in each pixel
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::simpleDEM()
{
	ROS_INFO_STREAM("[LiDARLandScene] Generating DEM with width " << image_width_ << " and height " << image_height_ << " at pixel size " << pixel_size_ << ".");

	// Generate actual DEM (in cloud and raster formats)
	cv::Mat img(image_height_, image_width_, CV_8UC1, cv::Scalar(0));
	pcl::PointCloud<pcl::PointXYZI>::Ptr dem_xyzi_cloud(new pcl::PointCloud<pcl::PointXYZI>());
	dem_image_.occupied_pixels.resize(image_width_, std::vector< bool >(image_height_, 0));
	for(int i=0; i<image_width_; i++)
		for(int j=0; j<image_height_; j++)
		{
			pcl::PointXYZI point;
			point.x = min_east_ + pixel_size_*(i+0.5);
			point.y = min_north_ + pixel_size_*(j+0.5);
			point.z = 0;
			point.intensity = 1;
			// Move on if there are no points in the pixel
			if((point_raster_list_[i][j]).size() < 1)
				continue;								
			// Find the minimum non-vegetation point in the pixel 
			float min_height = 10e10;
			for(int k=1; k<(point_raster_list_[i][j]).size(); k++)
			{
				// If next point in the pixel is the new minimum...
				if( input_cloud_->points[(point_raster_list_[i][j])[k]].z < min_height ) 							// Search for lowest point
					if ( input_cloud_->points[(point_raster_list_[i][j])[k]].curvature < normal_threshold_ ) 	// Exclude vegetation (high curvature)
						min_height = input_cloud_->points[(point_raster_list_[i][j])[k]].z;
			}
			// If we're still at the default min, no non-vegetation points exist in the pixel - move on
			if(min_height > 10e8)
				continue; 								// move on if there are no non-vegetation points in the pixel 
			point.z = min_height;
			// PCL Cloud
			dem_xyzi_cloud->points.push_back(point);
			// OpenCV Image Raster
			float depth_color;
			if(point.z < dem_image_.raster_height_min)
				depth_color = 0;
			else if(point.z > dem_image_.raster_height_max)
				depth_color = 255;
			else
				depth_color = floor(255*(point.z - dem_image_.raster_height_min)/(dem_image_.raster_height_max - dem_image_.raster_height_min));
			img.at<uchar>(image_height_-j-1,i) = depth_color;
			// For each populated pixel, record that it's been filled
			dem_image_.occupied_pixels[i][j] = true;
		}
	img.copyTo(dem_image_.image->image);

	// Populate DEM Normals
	pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> norm_est;
	KDP normals_tree (new KD ());
	norm_est.setSearchMethod (normals_tree);
	norm_est.setKSearch (15);
	norm_est.setInputCloud (dem_xyzi_cloud);
	norm_est.setViewPoint(0.0, 0.0, 1000);
	norm_est.compute(*dem_image_.cloud);
	pcl::copyPointCloud(*dem_xyzi_cloud, *dem_image_.cloud);
	ROS_DEBUG_STREAM("[LiDARLandScene]   Computed DEM and found normals. Size is " << dem_image_.cloud->points.size());
}

// -------- DEM Filtering --------
// DEM Filtering, currently just via extraction of contiguous Euclidean clusters
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::clusterFilterDEM()
{
	// DEM cluster extraction, several times, add em up
	// Set up cluster object with parameters
	pcl::EuclideanClusterExtraction<PointNormalType> ec;
	ec.setClusterTolerance(cluster_tolerance_); 
	ec.setMinClusterSize(20);
	ec.setMaxClusterSize (dem_image_.cloud->points.size()*2); 	// for safety, prevent maximum cluster size...
	// Set data inputs to cluster object
	KDNP clustering_tree (new KDN);
	clustering_tree->setInputCloud(dem_image_.cloud);
	ec.setSearchMethod(clustering_tree);
	ec.setInputCloud(dem_image_.cloud);
	// Perform cluster extraction
	std::vector<pcl::PointIndices> cluster_indices;
	ec.extract(cluster_indices);
	// Build new pointcloud based on cluster
	for(int i=0; i<cluster_indices[0].indices.size(); i++)
		dem_filtered_image_.cloud->points.push_back(dem_image_.cloud->points[cluster_indices[0].indices[i]]);
	// Debugging
	ROS_INFO_STREAM("Performed cluster extraction to remove lingering canopy data in DEM. Size after " << dem_filtered_image_.cloud->points.size() << " vs " << dem_image_.cloud->points.size() << " before.");
}

// -------- Water Extraction --------
// Water Extraction
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::waterEstimation()
{
	
}

// -------- Simple DSM --------
// Find highest points in each pixel
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::simpleDSM()
{
	ROS_INFO_STREAM("[LiDARLandScene] Generating DSM with width " << image_width_ << " and height " << image_height_ << " at pixel size " << pixel_size_ << ".");

	// Generate actual DSM (in cloud and raster formats)
	cv::Mat img(image_height_, image_width_, CV_8UC1, cv::Scalar(0));
	pcl::PointCloud<pcl::PointXYZI>::Ptr dsm_xyzi_cloud(new pcl::PointCloud<pcl::PointXYZI>());
	dsm_image_.occupied_pixels.resize(image_width_, std::vector< bool >(image_height_, 0));
	for(int i=0; i<image_width_; i++)
		for(int j=0; j<image_height_; j++)
		{
			pcl::PointXYZI point;
			point.x = min_east_ + pixel_size_*(i+0.5);
			point.y = min_north_ + pixel_size_*(j+0.5);
			point.z = 0;
			point.intensity = 1;
			// Move on if there are no points in the pixel
			if((point_raster_list_[i][j]).size() < 1)
				continue;								
			// Find the minimum non-vegetation point in the pixel 
			float max_height = -10e10;
			for(int k=1; k<(point_raster_list_[i][j]).size(); k++)
			{
				// If next point in the pixel is the new minimum...
				if( input_cloud_->points[(point_raster_list_[i][j])[k]].z > max_height ) 							// Search for lowest point
					if ( input_cloud_->points[(point_raster_list_[i][j])[k]].curvature < normal_threshold_ ) 	// Exclude vegetation (high curvature)
						max_height = input_cloud_->points[(point_raster_list_[i][j])[k]].z;
			}
			// If we're still at the default min, no non-vegetation points exist in the pixel - move on
			if(max_height < -10e8)
				continue; 								// move on if there are no non-vegetation points in the pixel 
			point.z = max_height;
			// PCL Cloud
			dsm_xyzi_cloud->points.push_back(point);
			// OpenCV Image Raster
			float depth_color;
			if(point.z < dsm_image_.raster_height_min)
				depth_color = 0;
			else if(point.z > dsm_image_.raster_height_max)
				depth_color = 255;
			else
				depth_color = floor(255*(point.z - dsm_image_.raster_height_min)/(dsm_image_.raster_height_max - dsm_image_.raster_height_min));
			img.at<uchar>(image_height_-j-1,i) = depth_color;
			// For each populated pixel, record that it's been filled
			dsm_image_.occupied_pixels[i][j] = true;
		}
	img.copyTo(dsm_image_.image->image);

	// Populate DEM Normals
	pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> norm_est;
	KDP normals_tree (new KD ());
	norm_est.setSearchMethod (normals_tree);
	norm_est.setKSearch (15);
	norm_est.setInputCloud (dsm_xyzi_cloud);
	norm_est.setViewPoint(0.0, 0.0, 1000);
	norm_est.compute(*dsm_image_.cloud);
	pcl::copyPointCloud(*dsm_xyzi_cloud, *dsm_image_.cloud);
	ROS_DEBUG_STREAM("[LiDARLandScene]   Computed DSM and found normals. Size is " << dsm_image_.cloud->points.size());
}

// -------- Simple CHM --------
// Determine height of non-ground structures in each pixel
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::simpleCHM()
{
	ROS_INFO_STREAM("[LiDARLandScene] Beginning estimation of CHM.");
	pcl::KdTreeFLANN<PointNormalType> tree;
	tree.setInputCloud(dem_filtered_image_.cloud);
	std::vector<int> nearest_indices;
	std::vector<float> nearest_dist_squareds;
	cv::Mat img(image_height_, image_width_, CV_8UC1, cv::Scalar(0));
	PCP chm_xyzi_cloud(new PC());
	for(int i=0; i<dsm_image_.cloud->points.size(); i++)
	{
		tree.nearestKSearch(dsm_image_.cloud->points[i], 1, nearest_indices, nearest_dist_squareds);

		int column 	= floor( (dsm_image_.cloud->points[i].x - min_east_) / pixel_size_ );
		int row 	= floor( (dsm_image_.cloud->points[i].y - min_north_) / pixel_size_ );

		float plant_height = dsm_image_.cloud->points[i].z - dem_filtered_image_.cloud->points[nearest_indices[0]].z;
		if(plant_height > 1)
		{
			// PCL Point Cloud
			PointType point;
			point.x = dsm_image_.cloud->points[i].x;
			point.y = dsm_image_.cloud->points[i].y;
			point.z = dsm_image_.cloud->points[i].z;
			point.intensity = plant_height;
			chm_xyzi_cloud->points.push_back(point);
			// OpenCV Image Raster
			float depth_color;
			if(point.z < chm_image_.raster_height_min)
				depth_color = 0;
			else if(point.z > chm_image_.raster_height_max)
				depth_color = 255;
			else
				depth_color = floor(255*(point.z - chm_image_.raster_height_min)/(chm_image_.raster_height_max - chm_image_.raster_height_min));
			img.at<uchar>(image_height_-row-1,column) = depth_color;
		}
	}
	img.copyTo(chm_image_.image->image);

	// Populate CHM Normals
	pcl::NormalEstimation<PointType, PointNormalType> norm_est;
	KDP normals_tree (new KD ());
	norm_est.setSearchMethod (normals_tree);
	norm_est.setKSearch (15);
	norm_est.setInputCloud (chm_xyzi_cloud);
	norm_est.setViewPoint(0.0, 0.0, 1000);
	norm_est.compute(*chm_image_.cloud);
	pcl::copyPointCloud(*chm_xyzi_cloud, *chm_image_.cloud);
	ROS_DEBUG_STREAM("[LiDARLandScene]   Computed CHM and found normals. Size is " << chm_image_.cloud->points.size());
}


// -------- Vegetation Cloud --------
// Generate a cloud containing only those points which appear to be vegetation 
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::vegetationCloud()
{
	pcl::KdTreeFLANN<PointNormalType> tree;
	tree.setInputCloud(dem_image_.cloud);
	std::vector<int> nearest_indices;
	std::vector<float> nearest_dist_squareds;
	int num_points = 0;
	for(int i=0; i<input_cloud_->points.size(); i++)
	{
		tree.nearestKSearch(input_cloud_->points[i], 1, nearest_indices, nearest_dist_squareds);

		float plant_height = input_cloud_->points[i].z - dem_image_.cloud->points[nearest_indices[0]].z;
		if(plant_height > 1)
		{
			VegetationPointType point;
			point.x = input_cloud_->points[i].x;
			point.y = input_cloud_->points[i].y;
			point.z = input_cloud_->points[i].z;
			point.normal_x = input_cloud_->points[i].normal_x;
			point.normal_y = input_cloud_->points[i].normal_y;
			point.normal_z = input_cloud_->points[i].normal_z;
			point.intensity = input_cloud_->points[i].intensity;
			point.vegetation_height = plant_height;
			
			vegetation_cloud_->points.push_back(point);
			num_points++;
		}

	}
	ROS_DEBUG_STREAM("[LiDARLandScene]   Generated vegetation cloud with size " << vegetation_cloud_->points.size());
}




// -------- Rasterize Cloud --------
// Take point cloud and generate a regular raster-gridded image
// For now, this assumes that each pixel starts out with only one point in the input cloud
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::rasterizeCloud(SceneImage image)
{
	cv::Mat img(image_height_, image_width_, CV_8UC1, cv::Scalar(0));
	for(int i=0; i<image.cloud->points.size(); i++)
	{
		int column 	= floor( (image.cloud->points[i].x - min_east_) / pixel_size_ );
		int row 	= floor( (image.cloud->points[i].y - min_north_) / pixel_size_ );

		image.cloud->points[i].z;

		float depth_color;
		if(image.cloud->points[i].z < image.raster_height_min)
			depth_color = 0;
		else if(image.cloud->points[i].z > image.raster_height_max)
			depth_color = 255;
		else
			depth_color = floor(255*(image.cloud->points[i].z - image.raster_height_min)/(image.raster_height_max - image.raster_height_min));
		img.at<uchar>(image_height_-row-1,column) = depth_color;
	}
	img.copyTo(image.image->image);
}


// -------- Paint Depth Image --------
// Colorize Depth Images
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::paintDepthImage()
{
	
}

// Reattach Canyons
// Hole Filling  



// -------- Publish --------
// Colorize Depth Images
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
LiDARLandScene<PointType, PointNormalType, VegetationPointType>::publish()
{
	// Publish Scene Frame to TF
	scene_transform_.header.stamp = ros::Time::now();
	scene_frame_broadcaster_.sendTransform(scene_transform_);

	if(publishers_initialized_)
	{
		// Publish ROS Messages - Input Cloud
		input_pub_.publish(input_msg_);
		// Publish ROS Messages - DEM
		dem_image_.cloud_pub.publish(dem_image_.cloud_msg);
		dem_image_.image_pub.publish(dem_image_.image_msg);
		// Publish ROS Messages - DEM Filtered
		dem_filtered_image_.cloud_pub.publish(dem_filtered_image_.cloud_msg);
		dem_filtered_image_.image_pub.publish(dem_filtered_image_.image_msg);
		// Publish ROS Messages - DSM
		dsm_image_.cloud_pub.publish(dsm_image_.cloud_msg);
		dsm_image_.image_pub.publish(dsm_image_.image_msg);
		// Publish ROS Messages - CHM
		chm_image_.cloud_pub.publish(chm_image_.cloud_msg);
		chm_image_.image_pub.publish(chm_image_.image_msg);
		// Publish ROS Messages - Vegetation
		vegetation_pub_.publish(vegetation_msg_);
	}
	else
		ROS_ERROR_STREAM("[LiDARLandScene] Warning - publishing requested, but publishers have not been initialized. No publishing will occur.");
}