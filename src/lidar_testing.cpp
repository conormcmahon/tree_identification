
#include <ros/ros.h>
#include "tree_identification/lidar_land_scene.h"
#include "tree_identification/vegetation_structure_evaluation.h"

int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "lidar_testing");

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    	ros::console::notifyLoggerLevelsChanged();

  	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    ros::NodeHandle nh;

    // *** Input Data ***
    //   Pull in all the input parameters from the ROS server (loaded via .yaml or .launch)
    std::string bag_name, bag_topic;
    nh.param<std::string>("lidar_testing/bag_name", bag_name, "/home/conor/UT_Thesis/ros_data/trees/Aerial Lidar/Subset Scenes/Hornsby/b3.bag");
    nh.param<std::string>("lidar_testing/bag_topic", bag_topic, "/subset_lidar");
    ROS_INFO_STREAM("[CHMGeneration] Loading clouds from bag files, using bag name: " << bag_name << " and topic name: " << bag_topic << ".");
	// Pixel Scale
    float pixel_size; 	// meters
	nh.param<float>("lidar_testing/dem_pixel_size", pixel_size, 1.0);
	// Altitudinal Limits
	float image_scale_min_height, image_scale_max_height;
	nh.param<float>("lidar_testing/image_scale_min_height", image_scale_min_height, 1.0);
	nh.param<float>("lidar_testing/image_scale_max_height", image_scale_max_height, 1.0);
	// Thresholding Limits for Vegetation
	float normal_threshold, intensity_threshold;
	nh.param<float>("lidar_testing/veg_normal_threshold", normal_threshold, 0.15); // minimum local curvature for vegetation
	nh.param<float>("lidar_testing/veg_intensity_threshold", intensity_threshold, 60); // not currently in use (maximum intensity for vegetation)
	// Euclidean Cluster Filtering for DEM
	float cluster_tolerance;
	nh.param<float>("lidar_testing/cluster_tolerance", cluster_tolerance, 2.0); // not currently in use (maximum intensity for vegetation)

	// *** Create Scene Object ***
    LiDARLandScene<pcl::PointXYZI, pcl::PointXYZINormal, pcl::VegetationPoint3> hornsby_scene;
    // Set Input Parameters
    hornsby_scene.inputROSBAG(bag_name, bag_topic);
    hornsby_scene.setPixelSize(pixel_size);
    hornsby_scene.setImageHeightLimits(image_scale_min_height, image_scale_max_height, image_scale_min_height, image_scale_max_height, image_scale_min_height, image_scale_max_height, image_scale_min_height, image_scale_max_height);
    hornsby_scene.setPublishingNamespace("hornsby",nh);
    hornsby_scene.setFrameNames("map", "hornsby_frame");
    hornsby_scene.setNormalThreshold(normal_threshold);
    hornsby_scene.setClusterTolerance(cluster_tolerance);

    // *** Actual Computation ***
    hornsby_scene.evaluateScene();
//hornsby_scene.loadLiDARScene("/home/conor/UT_Thesis/ros_data/trees/LiDARLandScene/Hornsby/");

/*
    std::vector<std::string> site_names;
    if(!nh.getParam("hornsby_lidar/site_list", site_names))
    {
      ROS_ERROR_STREAM("Warning - failed to get list of site names from parameter server. Exiting without further processing.");
      return -1;
    }
    std::vector< LiDARLandScene<pcl::PointXYZI, pcl::PointXYZINormal, pcl::VegetationPoint3> > site_list;
    std::vector< VegetationStructureEvaluation<pcl::PointXYZI, pcl::PointXYZINormal, pcl::VegetationPoint3> > site_vegetation_list;
    for(int i=0; i<site_names.size(); i++)
    {
      // *** Subset to Field Area ***
      float site_northing, site_easting, site_radius;
      nh.param<float>("hornsby_lidar/northing_temp", site_northing, 3345210.7 - 3344763.3);
      nh.param<float>("hornsby_lidar/easting_temp", site_easting, 629598.2 - 629918.42);
      nh.param<float>("hornsby_lidar/radius", site_radius, 40);
      LiDARLandScene<pcl::PointXYZI, pcl::PointXYZINormal, pcl::VegetationPoint3> subset_scene;
      subset_scene.setImageHeightLimits(image_scale_min_height, image_scale_max_height, image_scale_min_height, image_scale_max_height, image_scale_min_height, image_scale_max_height, image_scale_min_height, image_scale_max_height);
      subset_scene.setPublishingNamespace(site_names[i], nh);
      subset_scene.setFrameNames("map", "hornsby_frame");
      subset_scene.subsetLiDARScene(&hornsby_scene, site_northing, site_easting, site_radius);
      
      site_list.push_back(subset_scene);
      site_list[i].generateMsgs();

      VegetationStructureEvaluation<pcl::PointXYZI, pcl::PointXYZINormal, pcl::VegetationPoint3> vegetation;
      vegetation.inputLandScene(&site_list[0]);
      float average_height = vegetation.averageCanopyHeight();
      float average_height_occupied = vegetation.averageCanopyHeightOccupied();
      float canopy_cover_percentage = vegetation.canopyCoverage();
      ROS_INFO_STREAM(site_names[i] << " heights \n\t\t\t\t\tAverage: " << average_height << " \n\t\t\t\t\tOccupied Avg: " << average_height_occupied << " \n\t\t\t\t\tCoverage Amount: " << canopy_cover_percentage*100 << "%%");
      site_vegetation_list.push_back(vegetation);
    }
*/

    // *** Get Ouptuts ***
    hornsby_scene.generateMsgs(); 




  	// *** Save Land Scene Object ***
  	// Create a bunch of ROS Bag objects to save the generated overall land scene
  	//hornsby_scene.saveLandScene("/home/conor/UT_Thesis/ros_data/trees/LiDARLandScene/Hornsby/");

  	while(ros::ok())
  	{
  		hornsby_scene.publish();
//      for(int i=0; i<site_list.size(); i++)
//  		  site_list[i].publish();
  		ros::Duration(1.0).sleep();
  	}
}




/*   TODO

Hornsby Atta Stuff
1) Load list of hornsby points, process them all

LiDARLandSCene Stuff
2) Functions to estimate vegetation parameters within scene 
   - Vegetation height
   - Height variability 
   - Height distribution
   - Percent canopy cover / Percent ground penetration
3) Functions to estimate grade / hydrologic parameters within scene
   - Grade
   - Ruggedness 
   - Some hydrologic stuff
4) Water, Building recognition


Terrestrial LiDAR Data
1) Segment Trees
2) Classify Trees 
3) Color Projection? 


*/