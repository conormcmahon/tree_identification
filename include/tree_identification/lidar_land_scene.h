
#ifndef LIDAR_LAND_SCENE_H_
#define LIDAR_LAND_SCENE_H_

// Basic ROS Stuff
#include <ros/ros.h>
// Ros Bags (binary data storage format)
#include <rosbag/bag.h>
#include <rosbag/view.h>
// ROS Messages (communication between ROS nodes)
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
// ROS Transforms (spatial transform between difference LiDAR point cloud scenes)
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

// PCL Includes
//   Basic
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include "tree_identification/vegetation_point.h"
//   Filters
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

// OpenCV Includes
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Handling Directories in Linux
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

// ----------------------------------------------------------------
// ----------------------- LiDAR LAND SCENE -----------------------
// ----------------------------------------------------------------
// This class is used to process input Point Cloud data into a DEM, DSM and CHM
// Also outputs cloud containing all the vegetation points in input, with intensity based on local height
template <typename PointType, typename PointNormalType, typename VegetationPointType>
class LiDARLandScene
{
	// *** Useful Defines ***
	// These are actually required because of how PCL works
	//   Basic PointType Point Clouds and KDTrees
	typedef typename pcl::PointCloud<pcl::PointXYZI> PC;
	typedef typename pcl::PointCloud<pcl::PointXYZI>::Ptr PCP;
	typedef typename pcl::search::KdTree<PointType> KD;
	typedef typename pcl::search::KdTree<PointType>::Ptr KDP;
	//   PointNormalType Point Clouds and KdTrees
	typedef typename pcl::PointCloud<pcl::PointXYZINormal> PCN;
	typedef typename pcl::PointCloud<pcl::PointXYZINormal>::Ptr PCNP;
	typedef typename pcl::search::KdTree<PointNormalType> KDN;
	typedef typename pcl::search::KdTree<PointNormalType>::Ptr KDNP;
	//   VegetationPointType Point Clouds and KdTrees
	typedef typename pcl::PointCloud<VegetationPointType> VC;
	typedef typename pcl::PointCloud<VegetationPointType>::Ptr VCP;
	typedef typename pcl::search::KdTree<VegetationPointType> KDV;
	typedef typename pcl::search::KdTree<VegetationPointType>::Ptr KDVP;


// ----------------------- PUBLIC MEMBERS -----------------------
public: 
	LiDARLandScene();

	// *** Insert Input Data ***
	void inputLAS(std::string las_file_name);
	void inputPCD(std::string pcd_file_name);
	void inputROSBAG(std::string bag_file_name, std::string bag_topic);
	void inputTIF(std::string dem_file_name, std::string dsm_file_name);
	void loadLiDARScene(std::string parent_directory_path);

	// *** Subsetting ***
	void subsetLiDARScene(LiDARLandScene *input_scene, float northing_center, float easting_center, float radius);
	void subsetPointCloud(PCN input_cloud, PCNP output_cloud, float northing_center, float easting_center, float radius, float northing_transform, float easting_transform);
	void subsetPointCloud(VC input_cloud, VCP output_cloud, float northing_center, float easting_center, float radius, float northing_transform, float easting_transform);

	// *** Input Parameters ***
	void setPixelSize(float pixel_size);
	void setImageHeightLimits(float dem_min_height, float dem_max_height, float dem_filtered_min_height, float dem_filtered_max_height, float dsm_min_height, float dsm_max_height, float chm_min_height, float chm_max_height);
	void setFrameNames(std::string parent_frame, std::string scene_frame);
	void setPublishingNamespace(std::string publishing_namespace, ros::NodeHandle nh);
	void setNormalThreshold(float normal_threshold);
	void setClusterTolerance(float cluster_tolerance);
	void estimateImageHeightLimits();											

	// *** Calculation ***
	void evaluateScene();
	void paintDepthImages();
	
	// *** Get Outputs ***
	// Point Clouds (PCL)
	pcl::PointCloud<PointNormalType> getInputCloud();
	pcl::PointCloud<PointNormalType> getDEMCloud();
	pcl::PointCloud<PointNormalType> getDEMFilteredCloud();
	pcl::PointCloud<PointNormalType> getDSMCloud();
	pcl::PointCloud<PointNormalType> getCHMCloud();
	pcl::PointCloud<VegetationPointType> getVegetationCloud();
	// Raster Images (OpenCV)
	cv_bridge::CvImage getDEMImage();
	cv_bridge::CvImage getDEMFilteredImage();
	cv_bridge::CvImage getDSMImage();
	cv_bridge::CvImage getCHMImage();
	void getImages(cv_bridge::CvImagePtr dem_image, cv_bridge::CvImagePtr dsm_image, cv_bridge::CvImagePtr chm_image);
	// Get Transform
	geometry_msgs::TransformStamped getTransform();
	float getImageHeight();
	float getImageWidth();
	float getPixelSize();

	// *** Publish Outputs ***
	void generateMsgs();
	void getRasterCloudMsgs(sensor_msgs::PointCloud2 &dem_msg, sensor_msgs::PointCloud2 &dsm_msg, sensor_msgs::PointCloud2 &chm_msg);
	void getVegetationCloudMsg(sensor_msgs::PointCloud2 &vegetation_msg);
	void getImageMsgs(sensor_msgs::Image &dem_image, sensor_msgs::Image &dsm_image, sensor_msgs::Image &chm_image);
	void publish();

	// *** Save Land Scene ***
	void saveLandScene(std::string parent_directory_path);

	// *** Apply Color to LiDAR Scene
	void paintDepthImage();

	class SceneImage
	{
	public:
		// PCL Point Cloud Data
		PCNP cloud;
		// CV Raster Data
		cv_bridge::CvImagePtr image;
		std::vector< std::vector<bool> > occupied_pixels;
		float raster_height_min;
		float raster_height_max;
		// ROS Message Types
		sensor_msgs::PointCloud2 cloud_msg;
		sensor_msgs::Image image_msg;
		// ROS Publishers
		ros::Publisher cloud_pub;
		ros::Publisher image_pub;
	
		SceneImage():
			cloud(new PCN()),
			image(new cv_bridge::CvImage())
		{};
	};

// ----------------------- PRIVATE MEMBERS -----------------------
private:

	// *** PCL Clouds ***
	PCNP input_cloud_;		// Directly from LAS
	VCP vegetation_cloud_;	// All vegetation points in input

	// *** Frame Information ***
	tf2_ros::TransformBroadcaster scene_frame_broadcaster_; 
	geometry_msgs::TransformStamped scene_transform_;

	// *** Image Class Objects ***
	SceneImage dem_image_;
	SceneImage dem_filtered_image_;
	SceneImage dsm_image_;
	SceneImage chm_image_;

	// ROS Message Output - Vegetation
	//   Other cloud outputs are in the above Image Class objects
	sensor_msgs::PointCloud2 input_msg_;
	sensor_msgs::PointCloud2 vegetation_msg_;
	ros::Publisher input_pub_;
	ros::Publisher vegetation_pub_;
	bool publishers_initialized_;

	// *** Point List Images ***
	//   These are matrices the same size as the output raster images in which each entry
	//   contains a list of all the input points in that pixel. 
	std::vector< std::vector< std::vector<int> > > point_raster_list_;				// all input points, by pixel
	std::vector< std::vector< std::vector<int> > > vegetation_point_raster_list_; 	// vegetation points, by pixel 

	// *** Internal Parameters *** 
	//   Maximum curvature allowable for ground points
	float normal_threshold_; 		 
	//   Maximum distance to be included in ground cluster
	float cluster_tolerance_;

	// *** Image Limits *** 
	//   Size of one pixel in UTM units
	float pixel_size_;			
	//   Image limits in cardinal directions	
	float min_north_;
	float max_north_;
	float min_east_;
	float max_east_;
	//   Image size in pixels
	int image_width_;
	int image_height_;

	// *** Internal Calculations ***
	//   The following are all run internally after evaluateScene() is run
	void findSceneLimits(bool realign);
	void inputNormals();
	void splitToPixels();
	void simpleDEM();
	void clusterFilterDEM();
	void waterEstimation();
	void simpleDSM();
	void simpleCHM();
	void vegetationCloud();

	// *** Other Internal Functions ***
	// Break point cloud into a raster image
	void rasterizeCloud(SceneImage image);
	// Load previously saved data
	void loadCloudBag(std::string bag_path, std::string bag_topic, sensor_msgs::PointCloud2 &msg);
	void loadImageBag(std::string bag_path, std::string bag_topic, sensor_msgs::Image &msg);
	void loadTransformBag(std::string bag_path, std::string bag_topic, geometry_msgs::TransformStamped &msg);
	// Save calculated data
	void savePointCloud(std::string bag_path, std::string bag_topic, sensor_msgs::PointCloud2 cloud_msg)	;
	void saveImage(std::string bag_path, std::string bag_topic, sensor_msgs::Image image_msg, cv_bridge::CvImagePtr image);
	void saveTransform(std::string bag_path, std::string bag_topic, geometry_msgs::TransformStamped image_msg);
	// Check whether a directory exists, and create it if not (prior to saving in the directory)
	void createDirectory(std::string bag_path);
};


#endif // LIDAR_LAND_SCENE_H_