

#include "tree_identification/vegetation_structure_evaluation.h"

//  Density Metrics
// percent ground penetration
// histogram of canopy data
// Xth percentile of canopy

//  Height Metrics
// absolute canopy height
// stdev of max canopy height amongst neighbors


template class VegetationStructureEvaluation<pcl::PointXYZI, pcl::PointXYZINormal, pcl::VegetationPoint3>;

// --------------------------------------------------------------------------------------
// ------------------------------------ Basic Stuff -----------------------------------
template <typename PointType, typename PointNormalType, typename VegetationPointType>
VegetationStructureEvaluation<PointType, PointNormalType, VegetationPointType>::VegetationStructureEvaluation()
{

}

// --------------------------------------------------------------------------------------
// ------------------------------------ I/O Structure -----------------------------------
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
VegetationStructureEvaluation<PointType, PointNormalType, VegetationPointType>::inputLandScene(LiDARLandScene<PointType,PointNormalType,VegetationPointType> *land_scene)
{
	land_scene_ = land_scene;
}

// --------------------------------------------------------------------------------------
// ----------------------- Gross Measures of Vegetation Structure -----------------------

// -------- Average Canopy Height --------
// Average height of canopy across all pixels
template <typename PointType, typename PointNormalType, typename VegetationPointType> float
VegetationStructureEvaluation<PointType, PointNormalType, VegetationPointType>::averageCanopyHeight()
{
	float average_height_ = 0;
	for(int i=0; i<land_scene_->getCHMCloud().points.size(); i++)
		average_height_ += land_scene_->getCHMCloud().points[i].intensity;
	average_height_ /= land_scene_->getDSMCloud().points.size();
	return average_height_;
}
// -------- Average Canopy Height Occupied --------
// Average height of canopy within those pixels containing vegetation
template <typename PointType, typename PointNormalType, typename VegetationPointType> float
VegetationStructureEvaluation<PointType, PointNormalType, VegetationPointType>::averageCanopyHeightOccupied()
{
	float average_height_occupied_ = 0;
	for(int i=0; i<land_scene_->getCHMCloud().points.size(); i++)
		if(land_scene_->getCHMCloud().points[i].intensity > 0)
			average_height_occupied_ += land_scene_->getCHMCloud().points[i].intensity;
	average_height_occupied_ /= land_scene_->getCHMCloud().points.size();
	return average_height_occupied_;
}
// -------- Average Canopy Height Occupied --------
// Fraction of pixels in the scene with canopy above the input threshold
template <typename PointType, typename PointNormalType, typename VegetationPointType> float
VegetationStructureEvaluation<PointType, PointNormalType, VegetationPointType>::canopyCoverage(float height)
{
	return float(land_scene_->getCHMCloud().points.size()) / float(land_scene_->getDSMCloud().points.size());
}




// --------------------------------------------------------------------------------------
// --------------------- Pixelwise Measures of Vegetation Structure ---------------------
// -------- Percent Ground Penetration --------
// 
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
VegetationStructureEvaluation<PointType, PointNormalType, VegetationPointType>::percentGroundPenetration()
{
	for(int i=0; i<land_scene_->getImageHeight(); i++)
		for(int j=0; j<land_scene_->getImageWidth(); j++)
		{
			pcl::PointXYZI point;
			
			//point.x = land_scene_->
		}
}

// -------- Mean Vegetation Height --------
// 
template <typename PointType, typename PointNormalType, typename VegetationPointType> void
VegetationStructureEvaluation<PointType, PointNormalType, VegetationPointType>::meanVegetationHeight()
{

}
