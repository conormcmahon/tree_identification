
#ifndef VEGETATION_STRUCTURE_EVALUATION_H_
#define VEGETATION_STRUCTURE_EVALUATION_H_

#include <tree_identification/lidar_land_scene.h>

// ----------------------------------------------------------------
// ----------------------- LiDAR LAND SCENE -----------------------
// ----------------------------------------------------------------
// This class is used to process input Point Cloud data into a DEM, DSM and CHM
// Also outputs cloud containing all the vegetation points in input, with intensity based on local height
template <typename PointType, typename PointNormalType, typename VegetationPointType>
class VegetationStructureEvaluation
{
	public:
		VegetationStructureEvaluation();
		void inputLandScene(LiDARLandScene<PointType,PointNormalType,VegetationPointType> * land_scene);
		float averageCanopyHeight();
		float averageCanopyHeightOccupied();
		float canopyCoverage(float height=0);
		void percentGroundPenetration();
		void meanVegetationHeight();
	private:
		LiDARLandScene<PointType,PointNormalType,VegetationPointType> * land_scene_;

		float average_height_;
		float average_height_occupied_;


		// --- Point Clouds ---
		pcl::PointCloud<pcl::PointXYZI>::Ptr percent_ground_penetration_cloud;

		// --- CV Rasters ---
		cv_bridge::CvImagePtr percent_ground_penetration_image;

		// --- Point Cloud Messages ---
		sensor_msgs::PointCloud2 percent_ground_penetration_msg;

		// --- CV Raster Messages ---
		sensor_msgs::Image percent_ground_penetration_image_msg;
};

#endif //VEGETATION_STRUCTURE_EVALUATION_H_