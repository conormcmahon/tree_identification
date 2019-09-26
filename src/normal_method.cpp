

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_representation.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>


#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing.h>


#include <pcl/segmentation/extract_clusters.h>


#include <pcl/filters/crop_box.h>

#include <pcl/impl/instantiate.hpp>
#include <pcl/surface/impl/mls.hpp>


#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

/*
// Force PCL to instantiate intensity options for MLS
#include <pcl/impl/instantiate.hpp>
#include <pcl/surface/impl/mls.hpp>
PCL_INSTANTIATE_PRODUCT(MovingLeastSquares, ((pcl::PointXZYRGB)(pcl::PointXZYRGBNormal)) )

#ifdef _OPENMP
PCL_INSTANTIATE_PRODUCT(MovingLeastSquaresOMP, ((pcl::PointXZYRGB)(pcl::PointXZYRGBNormal)) )
#endif */

/*
	- Planes - all the same normal, continuous (euclidean cluster)
	- Cylinders - all normals perpendicular to the same axis, constant curvature radially, none longitudinally 
	- Planes - all normals the same, difference in normal direction is small 

	- Normalize Intensity
	- Estimate Normals
	- Estimate Normal Variation
	- Segment planes
		- choose a point with low normal variation, start new cloud
		- region-growing in euclidean space, don't keep ORDER points with DIFFERENT normals 
	- Segment cylinders
	- Estimate vegetation (normal randomness) 
	- Vertical cylinders with masses of





*/




void removePoint(int index, pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ptr)
{
	pcl::ExtractIndices<pcl::PointXYZINormal> extract;
	extract.setInputCloud(cloud_ptr);
	extract.setNegative(true);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	inliers->indices.push_back(index);
	extract.setIndices(inliers);

	extract.filter(*cloud_ptr);
}

float normalAngleDifference(pcl::PointXYZINormal source, pcl::PointXYZINormal target)
{
	float dot_product = source.normal_x*target.normal_x + source.normal_y*target.normal_y + source.normal_z*target.normal_z;
	float source_mag = sqrt(pow(source.normal_x,2) + pow(source.normal_y,2) + pow(source.normal_z,2));
	float target_mag = sqrt(pow(target.normal_x,2) + pow(target.normal_y,2) + pow(target.normal_z,2));
	return acos(dot_product/source_mag/target_mag);
}

float distanceAlongNormal(pcl::PointXYZINormal source, pcl::PointXYZINormal target)
{
	Eigen::Vector3f source_normal, distance_vector;
	source_normal << source.normal_x, source.normal_y, source.normal_z;
	distance_vector << target.x - source.x, target.y - source.y, target.z - source.z; 
	float source_mag = sqrt(pow(source.normal_x,2) + pow(source.normal_y,2) + pow(source.normal_z,2));
	source_normal /= source_mag;

	return source_normal.dot(distance_vector);
}

void growPlaneRegion(int initial_seed_index, pcl::PointCloud<pcl::PointXYZINormal>::Ptr region_cloud, pcl::PointCloud<pcl::PointXYZINormal>::Ptr normal_cloud, pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree, std::vector<bool> point_used)
{
	std::vector<int> nearest_indices;
	std::vector<float> nearest_dist_squareds;

	// Seeds are points in the region which meet region criteria; the region grows around these points
	// A point can be added to the region without being made a seed if it is identified as an EDGE
	std::vector<int> seed_indices;
	seed_indices.push_back(initial_seed_index);
	int seeds_processed = 0;

	// Consume seed list, while adding new seeds during processing, until region stops growing
	while(seeds_processed < seed_indices.size())
	{
		pcl::PointXYZINormal seed = normal_cloud->points[seed_indices[seeds_processed]];
		if ( tree->radiusSearch (seed, 0.1, nearest_indices, nearest_dist_squareds) > 0 )
		{
			for(int i=0; i<nearest_indices.size(); i++)
			{
				// Don't use points which have already been assigned to regions:
				if( !point_used[nearest_indices[i]] )
				{
					pcl::PointXYZINormal target = normal_cloud->points[nearest_indices[i]];
					if(nearest_dist_squareds[i] < 0.02)
					{
						if(distanceAlongNormal(seed, target) < 0.01)
						{
							// Add point to Region cloud
							region_cloud->push_back(target);
							// Remove point from list of candidates for new regions
							point_used[nearest_indices[i]] = true;
							if(target.intensity > 0.5)
								seed_indices.push_back(nearest_indices[i]);
						}
					}
					else break;
				}
			}
		}
		seeds_processed++;
	}
}



void simpleDEM(	pcl::PointCloud<pcl::PointXYZINormal>::Ptr input_cloud, 
				float pixel_size,
				float normal_threshold,
				float min_east,
				float max_east, 
				float min_north, 
				float max_north,
				float image_scale_min_height, float image_scale_max_height,
				std::vector< std::vector< std::vector<int> > > &raster_point_list, 
				std::vector< std::vector< pcl::PointXYZINormal > > &dem_raster,
				cv_bridge::CvImagePtr dem_image,
				std::vector< std::vector< bool> > &raster_density,
				pcl::PointCloud<pcl::PointXYZINormal>::Ptr dem_cloud )
{
	// Image size in pixels
	int image_width = ceil((max_east - min_east) / pixel_size); 		// size in pixels
	int image_height = ceil((max_north - min_north) / pixel_size); 	// size in pixels
	ROS_INFO_STREAM("[CHMGeneration] Generating DEM with width " << image_width << " and height " << image_height << " at pixel size " << pixel_size << ".");
	ROS_INFO_STREAM("[CHMGeneration]   Normal threshold: " << normal_threshold);

	// Generate gridded list of point indices from the cloud within each raster pixel
	for(int i=0; i<input_cloud->points.size(); i++)
	{
		// (i,j) Position of point within the raster grid
		int column 	= floor( (input_cloud->points[i].x - min_east) / pixel_size );
		int row 	= floor( (input_cloud->points[i].y - min_north) / pixel_size );
		// Add the point's index to the raster grid
		raster_point_list[column][row].push_back(i);
	}	
	// Generate actual DEM (in cloud and raster formats)
	cv::Mat img(image_height,image_width,CV_8UC3,cv::Scalar(0,0,0));
	pcl::PointCloud<pcl::PointXYZI>::Ptr dem_xyzi_cloud(new pcl::PointCloud<pcl::PointXYZI>());
	for(int i=0; i<image_width; i++)
		for(int j=0; j<image_height; j++)
		{
			pcl::PointXYZINormal point;
			point.x = min_east + pixel_size*(i+0.5);
			point.y = min_north + pixel_size*(j+0.5);
			point.z = 0;
			point.intensity = 1;
			dem_raster[i][j] = point;
			// Move on if there are no points in the pixel
			if((raster_point_list[i][j]).size() < 1)
				continue;								
			// Find the minimum non-vegetation point in the pixel 
			float min_height = 10e10;
			for(int k=1; k<(raster_point_list[i][j]).size(); k++)
			{
				input_cloud->points[(raster_point_list[i][j])[k]].z;
				// If next point in the pixel is the new minimum...
				if( input_cloud->points[(raster_point_list[i][j])[k]].z < min_height ) 							// Search for lowest point
					if ( input_cloud->points[(raster_point_list[i][j])[k]].intensity < normal_threshold ) 	// Exclude vegetation (curvature)
						min_height = input_cloud->points[(raster_point_list[i][j])[k]].z;
			}
			// If we're still at the default min, no non-vegetation points exist in the pixel - move on
			if(min_height > 10e8)
				continue; 								// move on if there are no non-vegetation points in the pixel 
			point.z = min_height;
			// PCL Cloud
			pcl::PointXYZI point_xyzi;
			point_xyzi.x = point.x;
			point_xyzi.y = point.y;
			point_xyzi.z = point.z;
			point_xyzi.intensity = point.intensity;
			dem_xyzi_cloud->points.push_back(point_xyzi);
			// Vector-grid of PCL Points
			dem_raster[i][j].z = point.z;
			// OpenCV Image Raster
			float depth_color;
			if(point.z < image_scale_min_height)
				depth_color = 0;
			else if(point.z > image_scale_max_height)
				depth_color = 255;
			else
				depth_color = floor(255*(point.z - image_scale_min_height)/(image_scale_max_height - image_scale_min_height));
			img.at<cv::Vec3b>(i,j)[2] = depth_color; 		// R
			img.at<cv::Vec3b>(i,j)[1] = depth_color;		// G
			img.at<cv::Vec3b>(i,j)[0] = depth_color;		// B
			// For each populated pixel, record that it's been filled
			raster_density[i][j] = true;
		}
	img.copyTo(dem_image->image);

	// Populate DEM Normals
	pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> norm_est;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr normals_tree (new pcl::search::KdTree<pcl::PointXYZI> ());
	norm_est.setSearchMethod (normals_tree);
	norm_est.setKSearch (15);
	norm_est.setInputCloud (dem_xyzi_cloud);
	norm_est.setViewPoint(0.0, 0.0, 1000);
	norm_est.compute(*dem_cloud);
	pcl::copyPointCloud(*dem_xyzi_cloud, *dem_cloud);
	ROS_DEBUG_STREAM("[CHMGeneration]   Computed DEM and found normals. Size is " << dem_cloud->points.size());
}

void simpleDSM(	pcl::PointCloud<pcl::PointXYZINormal>::Ptr input_cloud, 
				float pixel_size,
				float min_east,
				float max_east, 
				float min_north, 
				float max_north,
				float image_scale_min_height, float image_scale_max_height,
				std::vector< std::vector< std::vector<int> > > &raster_point_list, 
				std::vector< std::vector< pcl::PointXYZINormal > > &dsm_raster,
				cv_bridge::CvImagePtr dsm_image,
				std::vector< std::vector< bool> > &raster_density,
				pcl::PointCloud<pcl::PointXYZINormal>::Ptr dsm_cloud )
{
	// Image size in pixels
	int image_width = ceil((max_east - min_east) / pixel_size); 		// size in pixels
	int image_height = ceil((max_north - min_north) / pixel_size); 	// size in pixels
	ROS_INFO_STREAM("Generating DSM with width " << image_width << " and height " << image_height << " at pixel size " << pixel_size << ".");
	// Generate gridded list of point indices from the cloud within each raster pixel
	for(int i=0; i<input_cloud->points.size(); i++)
	{
		// Position of point within the raster grid
		int column 	= floor( (input_cloud->points[i].x - min_east) / pixel_size );
		int row 	= floor( (input_cloud->points[i].y - min_north) / pixel_size );
		// Add the point's index to the raster grid
		raster_point_list[column][row].push_back(i);
	}	
	// Generate actual DSM (in cloud and raster formats)
	cv::Mat img(image_height,image_width,CV_8UC3,cv::Scalar(0,0,0));
	pcl::PointCloud<pcl::PointXYZI>::Ptr dsm_xyzi_cloud(new pcl::PointCloud<pcl::PointXYZI>());
	for(int i=0; i<image_width; i++)
		for(int j=0; j<image_height; j++)
		{
			// Populate outputs	
			pcl::PointXYZINormal point;
			point.x = min_east + pixel_size*(i+0.5);
			point.y = min_north + pixel_size*(j+0.5);
			point.z = 0;
			point.intensity = 1;
			dsm_raster[i][j] = point;
			// Move on if there are no points in the pixel
			if((raster_point_list[i][j]).size() < 1)
				continue;								
			// Find the maximum non-vegetation point in the pixel
			float max_height = -10e10;
			for(int k=1; k<(raster_point_list[i][j]).size(); k++)
			{
				input_cloud->points[(raster_point_list[i][j])[k]].z;
				// If next point in the pixel is the new maximum...
				if( input_cloud->points[(raster_point_list[i][j])[k]].z > max_height ) 							// Search for lowest point
					max_height = input_cloud->points[(raster_point_list[i][j])[k]].z;
			}
			// If we're still at the default max, no non-vegetation points exist in the pixel - move on
			if(max_height < -10e8)
				continue; 				
			point.z = max_height;			
			// PCL Cloud 
			pcl::PointXYZI point_xyzi;
			point_xyzi.x = point.x;
			point_xyzi.y = point.y;
			point_xyzi.z = point.z;
			point_xyzi.intensity = point.intensity;
			dsm_xyzi_cloud->points.push_back(point_xyzi);
			// Vector-grid of PCL Points
			dsm_raster[i][j].z = point.z;
			// OpenCV Image Raster
			float depth_color;
			if(point.z < image_scale_min_height)
				depth_color = 0;
			else if(point.z > image_scale_max_height)
				depth_color = 255;
			else
				depth_color = floor(255*(point.z - image_scale_min_height)/(image_scale_max_height - image_scale_min_height));
			img.at<cv::Vec3b>(i,j)[2] = depth_color; 		// R
			img.at<cv::Vec3b>(i,j)[1] = depth_color;		// G
			img.at<cv::Vec3b>(i,j)[0] = depth_color;		// B
			// For each populated pixel, record that it's been filled 
			raster_density[i][j] = true;
		}
	img.copyTo(dsm_image->image);

	// Populate DSM Normals
	pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> norm_est;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr normals_tree (new pcl::search::KdTree<pcl::PointXYZI> ());
	norm_est.setSearchMethod (normals_tree);
	norm_est.setKSearch (15);
	norm_est.setInputCloud (dsm_xyzi_cloud);
	norm_est.setViewPoint(0.0, 0.0, 1000);
	norm_est.compute(*dsm_cloud);
	pcl::copyPointCloud(*dsm_xyzi_cloud, *dsm_cloud);
	ROS_DEBUG_STREAM("[CHMGeneration]   Computed DSM and found normals. Size is " << dsm_cloud->points.size());
}

void simpleCHM(pcl::PointCloud<pcl::PointXYZINormal>::Ptr dem_cloud,
			   pcl::PointCloud<pcl::PointXYZINormal>::Ptr dsm_cloud,
			   pcl::PointCloud<pcl::PointXYZINormal>::Ptr chm_cloud,
			   cv_bridge::CvImagePtr dem_image,
			   cv_bridge::CvImagePtr dsm_image,
			   cv_bridge::CvImagePtr chm_image)
{
	pcl::KdTreeFLANN<pcl::PointXYZINormal> tree;
	tree.setInputCloud(dem_cloud);
	std::vector<int> nearest_indices;
	std::vector<float> nearest_dist_squareds;
	pcl::PointCloud<pcl::PointXYZI>::Ptr chm_xyzi_cloud(new pcl::PointCloud<pcl::PointXYZI>());
	for(int i=0; i<dsm_cloud->points.size(); i++)
	{
		tree.nearestKSearch(dsm_cloud->points[i], 1, nearest_indices, nearest_dist_squareds);

		float plant_height = dsm_cloud->points[i].z - dem_cloud->points[nearest_indices[0]].z;
		if(plant_height > 1)
		{
			pcl::PointXYZI point;
			point.x = dsm_cloud->points[i].x;
			point.y = dsm_cloud->points[i].y;
			point.z = dsm_cloud->points[i].z;
			point.intensity = point.z - dem_cloud->points[nearest_indices[0]].z;
			chm_xyzi_cloud->points.push_back(point);
		}

	}
	// Populate CHM Normals
	pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> norm_est;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr normals_tree (new pcl::search::KdTree<pcl::PointXYZI> ());
	norm_est.setSearchMethod (normals_tree);
	norm_est.setKSearch (15);
	norm_est.setInputCloud (chm_xyzi_cloud);
	norm_est.setViewPoint(0.0, 0.0, 1000);
	norm_est.compute(*chm_cloud);
	pcl::copyPointCloud(*chm_xyzi_cloud, *chm_cloud);
	ROS_DEBUG_STREAM("[CHMGeneration]   Computed CHM and found normals. Size is " << chm_cloud->points.size());

}

void vegetationCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr dem_cloud,
			   		 pcl::PointCloud<pcl::PointXYZINormal>::Ptr normal_cloud,
			   		 pcl::PointCloud<pcl::PointXYZINormal>::Ptr vegetation_cloud)
{
	pcl::KdTreeFLANN<pcl::PointXYZINormal> tree;
	tree.setInputCloud(dem_cloud);
	std::vector<int> nearest_indices;
	std::vector<float> nearest_dist_squareds;
	int num_points = 0;
	for(int i=0; i<normal_cloud->points.size(); i++)
	{
		tree.nearestKSearch(normal_cloud->points[i], 1, nearest_indices, nearest_dist_squareds);

		float plant_height = normal_cloud->points[i].z - dem_cloud->points[nearest_indices[0]].z;
		if(plant_height > 1)
		{
			vegetation_cloud->points.push_back(normal_cloud->points[i]);
			vegetation_cloud->points[num_points].intensity -= dem_cloud->points[nearest_indices[0]].z;
			num_points++;
		}

	}
	ROS_DEBUG_STREAM("[CHMGeneration]   Generated vegetation cloud with size " << vegetation_cloud->points.size());

}


// This assumes that each point in the desired raster image is represented by ONE point in the input cloud
// If a pixel has multiple points, only the last one will be used
// A sparsity matrix of bools is returned also
void rasterizeCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
					float pixel_size,
					int image_width, int image_height,
					float min_east, float max_east, float min_north, float max_north,
					float image_scale_min_height, float image_scale_max_height,
					std::vector< std::vector<pcl::PointXYZINormal> > &point_raster,
					std::vector< std::vector<bool> > &raster_density,
					cv_bridge::CvImagePtr image)
{
	for(int column=0; column<image_width; column++)
		for(int row=0; row<image_height; row++)
		{
			pcl::PointXYZINormal point;
			point.x = min_east + float(float((column+0.5))*(max_east-min_east))/float(image_width);
			point.y = min_north + float(float((row+0.5))*(max_north-min_north))/float(image_height);
			point.z = 0;
			point_raster[column][row] = point;
		}
	cv::Mat img(image_height,image_width,CV_8UC3,cv::Scalar(0,0,0));
	for(int i=0; i<cloud->points.size(); i++)
	{
		pcl::PointXYZINormal point = cloud->points[i];
		// Position of point within the raster grid
		int column 	= floor( (point.x - min_east) / pixel_size );
		int row 	= floor( (point.y - min_north) / pixel_size );
		// Add the point's height to the CV Image
		float depth_color;
		if(point.z < image_scale_min_height)
			depth_color = 0;
		else if(point.z > image_scale_max_height)
			depth_color = 255;
		else
			depth_color = floor(255*(point.z - image_scale_min_height)/(image_scale_max_height - image_scale_min_height));
		img.at<cv::Vec3b>(column,row)[2] = depth_color; 	// R
		img.at<cv::Vec3b>(column,row)[1] = depth_color;		// G
		img.at<cv::Vec3b>(column,row)[0] = depth_color;		// B
		// Populate Point Raster
		point_raster[column][row] = point;
		// Populate Density Matrix
		raster_density[column][row] = true;
	}	
	img.copyTo(image->image);
	ROS_INFO_STREAM("Rasterizing Column/Row size: " << image_width << " " << image_height);
}



// ----------- InverseSquareGapFilling -----------
// Fill holes in a DEM by averaged values of neighbors weighted by their inverse distances
void inverseSquareGapFilling(float neighbor_distance, std::vector< std::vector<bool> > density,
						int image_width, int image_height,
						float image_scale_min_height, float image_scale_max_height,
						pcl::PointCloud<pcl::PointXYZINormal>::Ptr input_cloud,
						std::vector< std::vector<pcl::PointXYZINormal> > input_point_raster,
						cv_bridge::CvImagePtr input_image,
						pcl::PointCloud<pcl::PointXYZINormal>::Ptr filled_cloud, 
						std::vector< std::vector<pcl::PointXYZINormal> > filled_point_raster,
						cv_bridge::CvImagePtr filled_image)
{
	cv::Mat img(image_height,image_width,CV_8UC3,cv::Scalar(0,0,0));
	img = input_image->image;
	*filled_cloud = *input_cloud;
	filled_point_raster = input_point_raster;

	// Reorganize cloud into XY structure
	pcl::PointCloud<pcl::PointXY>::Ptr input_cloud_xy(new pcl::PointCloud<pcl::PointXY>());
	for(int i=0; i<input_cloud->points.size(); i++)
	{
		pcl::PointXY point;
		point.x = input_cloud->points[i].x;
		point.y = input_cloud->points[i].y;
		input_cloud_xy->points.push_back(point);
	}

	pcl::KdTreeFLANN<pcl::PointXY> tree;
	tree.setInputCloud(input_cloud_xy);
	int num_holes_initially = 0;
	int num_holes_left = 0;
	for(int column=0; column<image_width; column++)
	{
		for(int row=0; row<image_height; row++)
		{
			// Skip pixels which are already filled
			if(density[column][row])
			{
				// Populate Point Raster
				//filled_cloud->points.push_back(input_point_raster[column][row]);
				continue;
			}
			num_holes_initially++;
			// Get the point at this pixel
			pcl::PointXYZINormal point = input_point_raster[column][row];
			// Make a version with only XY data for neighbor search
			pcl::PointXY point_xy;
			point_xy.x = point.x;
			point_xy.y = point.y;
			// For each blank pixel, find all the nearest neighbors
			std::vector<int> nearest_point_indices;
			std::vector<float> nearest_dist_squareds;

			float height = 0;
			float sum_inverse_distance = 0;
			if(tree.radiusSearch(point_xy, neighbor_distance, nearest_point_indices, nearest_dist_squareds))
			{
				for(int k=0; k<nearest_point_indices.size(); k++)
				{
					height += input_cloud->points[nearest_point_indices[k]].z / nearest_dist_squareds[k];
					sum_inverse_distance += 1.0 / nearest_dist_squareds[k];
				}
				point.z = height/sum_inverse_distance;

				// *** Populate Outputs ***
				// Add the point's height to the CV Image
				float depth_color;
				if(point.z < image_scale_min_height)
					depth_color = 0;
				else if(point.z > image_scale_max_height)
					depth_color = 255;
				else
					depth_color = floor(255*(point.z - image_scale_min_height)/(image_scale_max_height - image_scale_min_height));
				img.at<cv::Vec3b>(column,row)[2] = depth_color; 	// R
				img.at<cv::Vec3b>(column,row)[1] = depth_color;		// G
				img.at<cv::Vec3b>(column,row)[0] = depth_color;		// B
				// Populate Point Raster
				filled_point_raster[column][row] = point;
				// Populate PCL Cloud
				filled_cloud->points.push_back(point);
			}
			else
			{
				num_holes_left++;
				//ROS_WARN_STREAM("During hole filling, failed to fill pixel at " << column << " x " << row);
			}
		}
	}
	img.copyTo(filled_image->image);
	ROS_INFO_STREAM("Rasterizing Column/Row size: " << image_width << " " << image_height);
	ROS_INFO_STREAM("Total pixels: " << image_width*image_height << "; Initial Holes: " << num_holes_initially << "; Holes Left: " << num_holes_left);
}



//template <typename PointXYZINormal>
void cloudLimits(float& min_east, float& max_east, float& min_north, float& max_north, pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_cloud)
{
	// Find range limits of the cloud
	min_east = 10e10;
	max_east = -10e10;
	min_north = 10e10;
	max_north = -10e10;
	for(int i=0; i<target_cloud->points.size(); i++)
	{
		pcl::PointXYZINormal point = target_cloud->points[i];
        if(point.x < min_east) 
            min_east = point.x;
        if(point.x > max_east) 
            max_east = point.x;
        if(point.y < min_north) 
            min_north = point.y;
        if(point.y > max_north) 
            max_north = point.y;
	}
	ROS_INFO_STREAM("Determined image limits. Easting: " << min_east << " to " << max_east << "; Northing: " << min_north << " to " << max_north);
}


int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "normal_method");

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    	ros::console::notifyLoggerLevelsChanged();

  	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    ros::NodeHandle nh;

    
    // ----------------------------- Pull in data -----------------------------
    std::string bag_name, bag_topic;
    nh.param<std::string>("normal_method/bag_name", bag_name, "/home/conor/ros_data/trees/Aerial Lidar/Subset Scenes/Hornsby/b3.bag");
    nh.param<std::string>("normal_method/bag_topic", bag_topic, "/subset_lidar");
    ROS_INFO_STREAM("[CHMGeneration] Loading clouds from bag files, using bag name: " << bag_name << " and topic name: " << bag_topic << ".");
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
          ROS_ERROR_STREAM("[CHMGeneration] Cloud caught for first cloud is null...");
    }
    input_bag.close();
    ROS_INFO_STREAM("[CHMGeneration] First cloud size: " << input_msg.height*input_msg.width);

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(input_msg, *input_cloud);



    // ----------------------------- Scale intensities -----------------------------
    float previous_exponent, new_exponent;
    nh.param<float>("normal_method/previous_exponent", previous_exponent, 0.45);
    nh.param<float>("normal_method/new_exponent", new_exponent, 2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr scaled_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for(int i=0; i<input_cloud->points.size(); i++)
    {
    	pcl::PointXYZI point = input_cloud->points[i];
    	float distance = sqrt( pow(point.x,2) + pow(point.y,2) + pow(point.z,2) );
    	point.intensity /= pow( distance, previous_exponent );
    	if(distance > 0.5)
    		point.intensity *= pow( distance, new_exponent );
    	else 
    	{
    		float multiplier = 1+2.916*distance;
    		point.intensity *= distance;
    	}
    	scaled_cloud->points.push_back(point);
    }
    sensor_msgs::PointCloud2 scaled_msg;
    pcl::toROSMsg(*scaled_cloud, scaled_msg);
    scaled_msg.header = input_msg.header;
    ROS_INFO_STREAM("[CHMGeneration] Finished scaling input cloud using old exponent " << previous_exponent << " and new exponent " << new_exponent);


    // ----------------------------- Voxelization -----------------------------
    pcl::PointCloud<pcl::PointXYZI>::Ptr voxelized_cloud(new pcl::PointCloud<pcl::PointXYZI>());
	float leaf_size;
	nh.param<float>("normal_method/leaf_size", leaf_size, 0.03);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
	vg.setInputCloud(scaled_cloud);
	vg.setLeafSize(leaf_size, leaf_size, leaf_size);
	vg.filter(*voxelized_cloud); 
	ROS_INFO_STREAM("[CHMGeneration] Performed voxelization on cloud - new cloud size is " << voxelized_cloud->points.size() << " using leaf size " << leaf_size);
	sensor_msgs::PointCloud2 voxelized_msg;
	pcl::toROSMsg(*voxelized_cloud, voxelized_msg);
	voxelized_msg.header = input_msg.header;


    // ----------------------------- Find Normal -----------------------------
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
    std::vector<int> index_source;
	pcl::removeNaNFromPointCloud(*voxelized_cloud, *voxelized_cloud, index_source);
	ROS_DEBUG_STREAM("[CHMGeneration]   Following NaN removal, new cloud size is " << voxelized_cloud->points.size());

	pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> norm_est;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
	norm_est.setSearchMethod (tree);
	norm_est.setKSearch (15);
	norm_est.setInputCloud (voxelized_cloud);
	norm_est.setViewPoint(0.0, 0.0, 1.2);

	// Perform Computation
	norm_est.compute (*normal_cloud);
	pcl::copyPointCloud(*voxelized_cloud, *normal_cloud);
	ROS_DEBUG_STREAM("[CHMGeneration]   Final PointNormal cloud size is " << normal_cloud->points.size());
	sensor_msgs::PointCloud2 normal_msg;
	pcl::toROSMsg(*normal_cloud, normal_msg);
	normal_msg.header = input_msg.header;



    // ----------------------------- Find Normal -----------------------------
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr normal_variation_cloud(new pcl::PointCloud<pcl::PointXYZINormal>()); 
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
		{
			float normal_x_mean = 0;
			float normal_y_mean = 0;
			float normal_z_mean = 0;
			for(int j=0; j<nearest_indices.size(); j++)
			{
				normal_x_mean += normal_cloud->points[nearest_indices[j]].normal_x;
				normal_y_mean += normal_cloud->points[nearest_indices[j]].normal_y;
				normal_z_mean += normal_cloud->points[nearest_indices[j]].normal_z;
			}
			normal_x_mean /= nearest_indices.size();
			normal_y_mean /= nearest_indices.size();
			normal_z_mean /= nearest_indices.size();

			float normal_stdev_x = 0;
			float normal_stdev_y = 0;
			float normal_stdev_z = 0;

			// Checking how similar nearby points are to the target point in normal value
			float angle_threshold = 0.122;
			int num_angles_below_threshold = 0;
			float mean_angle_difference = 0;
			Eigen::Vector3f normal_direction_source;
			normal_direction_source << source_point.normal_x, source_point.normal_y, source_point.normal_z; 
			for(int j=0; j<nearest_indices.size(); j++)
			{
				pcl::PointXYZINormal target_point = normal_cloud->points[nearest_indices[j]];

				normal_stdev_x += pow( target_point.normal_x - normal_x_mean, 2 );
				normal_stdev_y += pow( target_point.normal_y - normal_y_mean, 2 );
				normal_stdev_z += pow( target_point.normal_z - normal_z_mean, 2 );
				
				Eigen::Vector3f normal_direction_target;
				normal_direction_target << target_point.normal_x, target_point.normal_y, target_point.normal_z;
				float dot_product = source_point.normal_x*target_point.normal_x + source_point.normal_y*target_point.normal_y + source_point.normal_z*target_point.normal_z;  //normal_direction_source.dot(normal_direction_target);
				float source_normal_mag = sqrt(pow(source_point.normal_x,2) + pow(source_point.normal_y,2) + pow(source_point.normal_z,2));
				float target_normal_mag = sqrt(pow(target_point.normal_x,2) + pow(target_point.normal_y,2) + pow(target_point.normal_z,2));
				float angle_offset = acos(dot_product/source_normal_mag/target_normal_mag);
				if (angle_offset < angle_threshold)
					num_angles_below_threshold ++;
				mean_angle_difference += angle_offset; 
			}
			normal_stdev_x /= (nearest_indices.size()-1);
			normal_stdev_y /= (nearest_indices.size()-1);
			normal_stdev_z /= (nearest_indices.size()-1);

			point_n.normal_x = sqrt(normal_stdev_x);
			point_n.normal_y = sqrt(normal_stdev_y);
			point_n.normal_z = sqrt(normal_stdev_z);

			point_n.intensity = sqrt( pow(normal_stdev_x,2) + pow(normal_stdev_y,2) + pow(normal_stdev_z,2) );
		
			point_n.curvature = 1 - float(num_angles_below_threshold) / float(nearest_indices.size());
			mean_angle_difference /= nearest_indices.size();
			//point_n.intensity = mean_angle_difference;
		}
		else 
			continue;

		normal_variation_cloud->points.push_back(point_n);
	} 
	sensor_msgs::PointCloud2 normal_variation_msg;
	pcl::toROSMsg(*normal_variation_cloud, normal_variation_msg);
	normal_variation_msg.header = input_msg.header;


	float normal_threshold, intensity_threshold;
	nh.param<float>("normal_method/veg_normal_threshold", normal_threshold, 0.15);
	nh.param<float>("normal_method/veg_intensity_threshold", intensity_threshold, 60);
/*	pcl::PointCloud<pcl::PointXYZI>::Ptr non_vegetation(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr vegetation(new pcl::PointCloud<pcl::PointXYZI>());


	sensor_msgs::PointCloud2 non_vegetation_msg;
	sensor_msgs::PointCloud2 vegetation_msg;
	pcl::toROSMsg(*non_vegetation, non_vegetation_msg);
	pcl::toROSMsg(*vegetation, vegetation_msg);
	non_vegetation_msg.header.frame_id = input_msg.header.frame_id;
	vegetation_msg.header.frame_id = input_msg.header.frame_id;
	ROS_INFO_STREAM("Segmented vegetation! " << vegetation->points.size() << " points vegetation, and " << non_vegetation->points.size() << " points non-vegetative matter. " << float(vegetation->points.size())/non_vegetation->points.size()*100 << "% of points are plant.");


	// Set vegetation "intensity" based on height over nearest DEM points
	pcl::KdTreeFLANN<pcl::PointXYZINormal> vegetation_tree;

	for(int i=0; i<vegetation->points.size(); i++)
	{

	}



	// Filter Normal Variation cloud to keep only the 'flat' regions (< 50% within 7deg of source point)
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr flat_points(new pcl::PointCloud<pcl::PointXYZINormal>);
	for(int i=0; i<normal_variation_cloud->points.size(); i++)
		if(normal_variation_cloud->points[i].curvature < 0.5)
			flat_points->points.push_back(normal_variation_cloud->points[i]);

	// Region-growing to find planar regions in image
	std::vector< pcl::PointCloud<pcl::PointXYZINormal> > ground_planes; 
	std::vector< pcl::PointCloud<pcl::PointXYZINormal> > wall_planes;	*/
	// 
/*	float depth_deviation_threshold = 0.02;
	int seed_index = 0;
	std::vector<bool> point_used;
	for(int i=0; i<normal_variation_cloud->points.size(); i++)
		point_used.push_back(false);
	while( flat_points->points.size() > 0 )
	{
		// Start point for region
		pcl::PointXYZINormal region_start_point = flat_points->points[seed_index];
		removePoint(0, flat_points);
		// Planar Region Cloud
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr planar_cloud;

		planar_cloud->push_back(region_start_point);

		pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree;
		tree->setInputCloud(normal_cloud);

		growPlaneRegion(seed_index, planar_cloud, normal_variation_cloud, tree, point_used);

		//for(int i=0; i<)
		seed_index++;
	} 
*/

	// ----------------------------------------
	// *** Determine Cloud Limits ***
	// ----------------------------------------
	// Lat/Lon Limits
	ROS_INFO_STREAM("Finding DEM for cloud");
	float min_east, max_east, min_north, max_north;
	cloudLimits(min_east, max_east, min_north, max_north, normal_cloud);
	// Altitudinal Limits
	float image_scale_min_height, image_scale_max_height;
	nh.param<float>("normal_method/image_scale_min_height", image_scale_min_height, 1.0);
	nh.param<float>("normal_method/image_scale_max_height", image_scale_max_height, 1.0);

	// ----------------------------------------
	// *** Build Fine Spatial Raster ***
	float pixel_size; 	// meters
	nh.param<float>("normal_method/dem_pixel_size", pixel_size, 1.0);
	int image_width = ceil((max_east - min_east) / pixel_size); 		// size in pixels
	int image_height = ceil((max_north - min_north) / pixel_size); 	// size in pixels
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr dem_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
	std::vector< std::vector< std::vector<int> > >  raster_point_list(image_width, std::vector< std::vector<int> >(image_height, std::vector<int>(0)));
	std::vector< std::vector< pcl::PointXYZINormal > >  dem_raster(image_width, std::vector< pcl::PointXYZINormal >(image_height)); 
	cv_bridge::CvImagePtr dem_image(new cv_bridge::CvImage);
	std::vector< std::vector< bool > >  raster_density(image_width, std::vector< bool >(image_height, false)); 
	// Create first set of rasters and stuff
	simpleDEM(	normal_variation_cloud, pixel_size, normal_threshold,
				min_east, max_east, min_north, max_north,
				image_scale_min_height, image_scale_max_height,
				raster_point_list, dem_raster, dem_image, raster_density, dem_cloud );

	// ----------------------------------------
	// *** Build Coarse Spatial Raster ***
	float pixel_size_coarse; 	// meters
	nh.param<float>("normal_method/dem_pixel_size_coarse", pixel_size_coarse, 15);
	int image_width_coarse = ceil((max_east - min_east) / pixel_size_coarse); 		// size in pixels
	int image_height_coarse = ceil((max_north - min_north) / pixel_size_coarse); 	// size in pixels
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr dem_cloud_coarse(new pcl::PointCloud<pcl::PointXYZINormal>());
	std::vector< std::vector< std::vector<int> > >  raster_point_list_coarse(image_width_coarse, std::vector< std::vector<int> >(image_height_coarse, std::vector<int>(0)));
	std::vector< std::vector< pcl::PointXYZINormal > >  dem_raster_coarse(image_width_coarse, std::vector< pcl::PointXYZINormal >(image_height_coarse)); 
	cv_bridge::CvImagePtr dem_image_coarse(new cv_bridge::CvImage);
	std::vector< std::vector< bool > >  raster_density_coarse(image_width_coarse, std::vector< bool >(image_height_coarse, false)); 
	// Create first set of rasters and stuff
	simpleDEM(	normal_variation_cloud, pixel_size_coarse, normal_threshold, 
				min_east, max_east, min_north, max_north,
				image_scale_min_height, image_scale_max_height,
				raster_point_list_coarse, dem_raster_coarse, dem_image_coarse, raster_density_coarse, dem_cloud_coarse );

	// Iterate through DEM and remove points that seem like height outliers
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr filtered_dem_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
	//pcl::KdTreeFLANN<pcl::PointXYZINormal> dem_tree;
	//dem_tree.setInputCloud(dem_cloud);
	//std::vector<int> dem_nearest_indices;
	//std::vector<float> dem_nearest_distances_sqrd;
	for(int i=0; i<image_width; i++)
	{
		for(int j=0; j<image_height; j++)
		{
			// Skip this raster grid point if it's empty in the fine image
			if(!raster_density[i][j])
				continue;
			float coarse_easting = i*pixel_size/pixel_size_coarse;
			float coarse_northing = j*pixel_size/pixel_size_coarse;
			// Skip this raster grid point if it's empty in the fine image
			if(!raster_density_coarse[coarse_easting][coarse_northing])
				continue;
			// Reject fine-DEM points which are substantially above the coarse-DEM as likely vegetation
			if( dem_raster[i][j].z - dem_raster_coarse[coarse_easting][coarse_northing].z < 1.0 )
				filtered_dem_cloud->points.push_back(dem_raster[i][j]);
		}
		/*bool acceptable = true;
		pcl::PointXYZINormal point = dem_cloud->points[i];
		if(dem_tree.radiusSearch(point, 6.0, dem_nearest_indices, dem_nearest_distances_sqrd))
		{
			for(int j=0; j<dem_nearest_indices.size(); j++)
			{
				if( point.z - dem_cloud->points[dem_nearest_indices[j]].z > 1.0 )
				{
					acceptable = false;
					break;
				}
			}
			if(acceptable)
				filtered_dem_cloud->points.push_back(point);
		} */
	}

	sensor_msgs::PointCloud2 filtered_dem_msg;
	pcl::toROSMsg(*filtered_dem_cloud, filtered_dem_msg);
	filtered_dem_msg.header.frame_id = input_msg.header.frame_id;

	sensor_msgs::PointCloud2 dem_msg;
	pcl::toROSMsg(*dem_cloud, dem_msg);
	dem_msg.header.frame_id = input_msg.header.frame_id;
	ROS_INFO_STREAM("DEM found! Image size: " << image_width << "x" << image_height);

	sensor_msgs::Image dem_image_msg;
	dem_image->toImageMsg(dem_image_msg);
	dem_image_msg.encoding = sensor_msgs::image_encodings::BGR8;

	// --------------------------------------------
	// Euclidean Clustering
	// --------------------------------------------
	// Set up cluster object with parameters
	pcl::EuclideanClusterExtraction<pcl::PointXYZINormal> ec;
	float cluster_tolerance;
	nh.param<float>("normal_method/cluster_tolerance", cluster_tolerance, 2.0);
	ec.setClusterTolerance(cluster_tolerance); 
	ec.setMinClusterSize(20);
	ec.setMaxClusterSize (normal_cloud->points.size()*2); 	// for safety, prevent maximum cluster size...
	// Set data inputs to cluster object
	pcl::search::KdTree<pcl::PointXYZINormal>::Ptr clustering_tree (new pcl::search::KdTree<pcl::PointXYZINormal>);
	clustering_tree->setInputCloud(dem_cloud);
	ec.setSearchMethod(clustering_tree);
	ec.setInputCloud(dem_cloud);
	// Perform cluster extraction
	std::vector<pcl::PointIndices> cluster_indices;
	ec.extract(cluster_indices);
	// Build new pointcloud based on cluster
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr dem_cluster_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
	for(int i=0; i<cluster_indices[0].indices.size(); i++)
		dem_cluster_cloud->points.push_back(dem_cloud->points[cluster_indices[0].indices[i]]);
	// Output msg format
	sensor_msgs::PointCloud2 dem_cluster_msg;
	pcl::toROSMsg(*dem_cluster_cloud, dem_cluster_msg);
	dem_cluster_msg.header.frame_id = input_msg.header.frame_id;
	ROS_INFO_STREAM("Performed cluster extraction to remove lingering canopy data in DEM. Size after " << dem_cluster_cloud->points.size() << " vs " << dem_cloud->points.size() << " before.");

	// --------------------------------------------


	// --------------------------------------------
	// Re-attachment of Canyons
	// --------------------------------------------	
	// Create cloud of the DEM points removed during clustering operation
	//   Set up extract_indices object
	pcl::ExtractIndices<pcl::PointXYZINormal> extract_indices;
	extract_indices.setInputCloud(dem_cloud);
	pcl::IndicesPtr cluster_indices_ptr(&cluster_indices[0].indices);
	extract_indices.setIndices(cluster_indices_ptr);
	//   Actually extract the points
	extract_indices.setNegative(true);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr false_DEM_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
	extract_indices.filter(*false_DEM_cloud);
	// Iterate through removed points, add them back in if they are BELOW the local DEM
	pcl::KdTreeFLANN<pcl::PointXYZINormal> reattachment_tree;
	reattachment_tree.setInputCloud(dem_cluster_cloud);
	float reattachment_search_radius;
	nh.param<float>("normal_method/reattachment_search_radius", reattachment_search_radius, 3.0);
	std::vector<int> nearest_indices;
	std::vector<float> nearest_dist_squareds;
	// New cloud for raster with canyons reattached
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr reattached_dem_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
	*reattached_dem_cloud = *dem_cluster_cloud;
	for(int i=0; i<false_DEM_cloud->points.size(); i++)
	{
		bool reattach = false;
		pcl::PointXYZINormal source_point = false_DEM_cloud->points[i];
		if(reattachment_tree.radiusSearch(false_DEM_cloud->points[i], reattachment_search_radius, nearest_indices, nearest_dist_squareds))
			for(int j=0; j<nearest_indices.size(); j++)
				if(source_point.z < dem_cluster_cloud->points[nearest_indices[j]].z)
				{
					reattached_dem_cloud->points.push_back(source_point);
					break;
				}
	}
	// Output ROS msg
	sensor_msgs::PointCloud2 reattached_dem_msg;
	pcl::toROSMsg(*reattached_dem_cloud, reattached_dem_msg);
	reattached_dem_msg.header.frame_id = input_msg.header.frame_id;
	ROS_INFO_STREAM("Performed reattachment of canyons. " << reattached_dem_cloud->points.size()-dem_cluster_cloud->points.size() << " points reattached.");

	std::vector< std::vector<bool> > raster_density_dem_reattached(image_width, std::vector<bool>(image_height, false));
	std::vector< std::vector<pcl::PointXYZINormal> > reattached_dem_point_raster(image_width, std::vector<pcl::PointXYZINormal>(image_height));
	cv_bridge::CvImagePtr reattached_dem_image(new cv_bridge::CvImage);
	rasterizeCloud(reattached_dem_cloud,
				   pixel_size, image_width, image_height,
				   min_east, max_east, min_north, max_north,
				   image_scale_min_height, image_scale_max_height,
				   reattached_dem_point_raster, raster_density_dem_reattached,
				   reattached_dem_image);
	sensor_msgs::Image reattached_dem_image_msg;
	reattached_dem_image->toImageMsg(reattached_dem_image_msg);
	reattached_dem_image_msg.encoding = sensor_msgs::image_encodings::BGR8;

	// --------------------------------------------
	// Gap-filling in the DEM via interpolation. 
	// --------------------------------------------	
	// For each point in the DEM, pick the nearest neighbor in each direction and double-interpolate between them. 
	// Alternatively, just nearest neighbors? 
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr dem_filled_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
	std::vector< std::vector<pcl::PointXYZINormal> > dem_filled_raster(image_width, std::vector<pcl::PointXYZINormal>(image_height));
	cv_bridge::CvImagePtr dem_filled_image(new cv_bridge::CvImage);
	float gap_filling_radius;
	nh.param<float>("normal_method/gap_filling_radius", gap_filling_radius, 10);
	inverseSquareGapFilling(gap_filling_radius, raster_density_dem_reattached, 
							image_width, image_height, 
							image_scale_min_height, image_scale_max_height,
							reattached_dem_cloud, reattached_dem_point_raster, reattached_dem_image,
							dem_filled_cloud, dem_filled_raster, dem_filled_image);
	// Message Ouptut - PCL CLoud
	sensor_msgs::PointCloud2 dem_filled_msg;
	pcl::toROSMsg(*dem_filled_cloud, dem_filled_msg);
	dem_filled_msg.header.frame_id = input_msg.header.frame_id;
	// Message Output - OpenCV Image
	sensor_msgs::Image dem_filled_image_msg;
	dem_filled_image->toImageMsg(dem_filled_image_msg);
	dem_filled_image_msg.encoding = sensor_msgs::image_encodings::BGR8;

	// --------------------------------------------
	// Generate DSM and CHM  
	// --------------------------------------------	

	// --------------- Generate DSM --------------- 
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr dsm_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
	std::vector< std::vector< std::vector<int> > >  raster_point_list_dsm(image_width, std::vector< std::vector<int> >(image_height, std::vector<int>(0)));
	std::vector< std::vector< pcl::PointXYZINormal > >  dsm_raster(image_width, std::vector< pcl::PointXYZINormal >(image_height)); 
	cv_bridge::CvImagePtr dsm_image(new cv_bridge::CvImage);
	std::vector< std::vector< bool > >  raster_density_dsm(image_width, std::vector< bool >(image_height, false)); 
	// Create first set of rasters and stuff
	simpleDSM(	normal_variation_cloud, pixel_size,
				min_east, max_east, min_north, max_north,
				image_scale_min_height, image_scale_max_height,
				raster_point_list_dsm, dsm_raster, dsm_image, raster_density_dsm, dsm_cloud );
	// Message Output - PCL Cloud	
	sensor_msgs::PointCloud2 dsm_msg;
	pcl::toROSMsg(*dsm_cloud, dsm_msg);
	dsm_msg.header.frame_id = input_msg.header.frame_id;
	// Message Output - OpenCV Image	
	sensor_msgs::Image dsm_image_msg;
	dsm_image->toImageMsg(dsm_image_msg);
	dsm_image_msg.encoding = sensor_msgs::image_encodings::BGR8;

	// --------------- Generate CHM --------------- 
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr chm_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
	cv_bridge::CvImagePtr chm_image(new cv_bridge::CvImage);
	simpleCHM(dem_filled_cloud, dsm_cloud, chm_cloud, dem_image, dsm_image, chm_image);
	sensor_msgs::PointCloud2 chm_msg;
	pcl::toROSMsg(*chm_cloud, chm_msg);
	chm_msg.header.frame_id = input_msg.header.frame_id;

	// --------------- Generate Vegetation Cloud --------------- 
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr vegetation_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
	vegetationCloud(dem_filled_cloud, normal_cloud, vegetation_cloud);
	sensor_msgs::PointCloud2 vegetation_msg;
	pcl::toROSMsg(*vegetation_cloud, vegetation_msg);
	vegetation_msg.header.frame_id = input_msg.header.frame_id;

/*
	// --------------- MLS Resampling --------------- 
	pcl::MovingLeastSquares<pcl::PointXYZINormal, pcl::PointXYZINormal> mls;

    mls.setComputeNormals(false);
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZINormal, pcl::PointXYZINormal>::UpsamplingMethod::VOXEL_GRID_DILATION);
    mls.setDilationVoxelSize(1.0);
    mls.setDilationIterations(1);
    mls.setInputCloud(reattached_dem_cloud);
    mls.setPolynomialFit(true);  
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr mls_tree(new pcl::search::KdTree<pcl::PointXYZINormal>());
    mls.setSearchMethod(mls_tree);
    float mls_radius;
    nh.param<float>("feature_test/mls_radius", mls_radius, 3);
    mls.setSearchRadius(mls_radius);
    ROS_INFO_STREAM("[CHMGeneration] Initialized MLS Object and set parameters. Input cloud size: " << reattached_dem_cloud->points.size());

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr dem_smoothed(new pcl::PointCloud<pcl::PointXYZINormal>);
    mls.process(*dem_smoothed);
    ROS_INFO_STREAM("[CHMGeneration] Performed MLS smoothing! New cloud size: " << dem_smoothed->points.size());
    sensor_msgs::PointCloud2 dem_smoothed_msg;
    pcl::toROSMsg(*dem_smoothed, dem_smoothed_msg);
    dem_smoothed_msg.header = input_msg.header;
*/




	ROS_INFO_STREAM("[CHMGeneration] All math finished, running publishers.");

    // ----------------------------- Publishing -----------------------------
    ros::Publisher input_pub = nh.advertise<sensor_msgs::PointCloud2>("normal_method/input_cloud", 1);
    ros::Publisher scaled_pub = nh.advertise<sensor_msgs::PointCloud2>("normal_method/scaled_cloud", 1);
    ros::Publisher voxelized_pub = nh.advertise<sensor_msgs::PointCloud2>("normal_method/voxelized_cloud", 1);
    ros::Publisher normal_pub = nh.advertise<sensor_msgs::PointCloud2>("normal_method/normal_cloud", 1);
/*    ros::Publisher normal_variation_pub = nh.advertise<sensor_msgs::PointCloud2>("normal_method/normal_variation_cloud", 1);
    ros::Publisher non_vegetation_pub = nh.advertise<sensor_msgs::PointCloud2>("normal_method/non_vegetation", 1);
    ros::Publisher vegetation_pub = nh.advertise<sensor_msgs::PointCloud2>("normal_method/vegetation", 1); */
    ros::Publisher dem_pub = nh.advertise<sensor_msgs::PointCloud2>("normal_method/dem", 1);
    ros::Publisher dem_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("normal_method/dem_filtered", 1);
    ros::Publisher dem_cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("normal_method/dem_cluster", 1);
    ros::Publisher reattached_dem_pub = nh.advertise<sensor_msgs::PointCloud2>("normal_method/dem_reattached", 1);
    ros::Publisher dem_filled_pub = nh.advertise<sensor_msgs::PointCloud2>("normal_method/dem_filled", 1);
    ros::Publisher dsm_pub = nh.advertise<sensor_msgs::PointCloud2>("normal_method/dsm", 1);
    ros::Publisher chm_pub = nh.advertise<sensor_msgs::PointCloud2>("normal_method/chm", 1);
    ros::Publisher vegetation_pub = nh.advertise<sensor_msgs::PointCloud2>("normal_method/vegetation", 1);

    ros::Publisher dem_image_pub = nh.advertise<sensor_msgs::Image>("normal_method/dem_image", 1);
    ros::Publisher dsm_image_pub = nh.advertise<sensor_msgs::Image>("normal_method/dsm_image", 1);
    ros::Publisher dem_image_reattached_pub = nh.advertise<sensor_msgs::Image>("normal_method/dem_image_reattached", 1);
    ros::Publisher dem_image_filled_pub = nh.advertise<sensor_msgs::Image>("normal_method/dem_image_filled", 1);

    //ros::Publisher dem_smoothed_pub = nh.advertise<sensor_msgs::PointCloud2>("normal_method/dem_smoothed", 1);



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dem_color(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr chm_color(new pcl::PointCloud<pcl::PointXYZRGB>());
    for(int i=0; i<dem_cloud->points.size(); i++)
    {
    	pcl::PointXYZRGB point;
    	point.x = dem_cloud->points[i].x;
    	point.y = dem_cloud->points[i].y;
    	point.z = dem_cloud->points[i].z;
    	point.r = dem_cloud->points[i].intensity;
    	point.g = dem_cloud->points[i].intensity;
    	point.b = dem_cloud->points[i].intensity;
    	dem_color->points.push_back(point);
    }
    for(int i=0; i<chm_cloud->points.size(); i++)
    {
    	pcl::PointXYZRGB point;
    	point.x = chm_cloud->points[i].x;
    	point.y = chm_cloud->points[i].y;
    	point.z = chm_cloud->points[i].z;
    	point.r = chm_cloud->points[i].intensity;
    	point.g = chm_cloud->points[i].intensity;
    	point.b = chm_cloud->points[i].intensity;
    	chm_color->points.push_back(point);
    }

	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_dem(dem_color);
	viewer->addPointCloud<pcl::PointXYZRGB> (dem_color, rgb_dem, "sample cloud");
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_chm(chm_color);
	viewer->addPointCloud<pcl::PointXYZRGB> (chm_color, rgb_chm, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();


	while (!viewer->wasStopped ())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds(1000000));
	}

    while(ros::ok())
    {
    	input_pub.publish(input_msg);
    	scaled_pub.publish(scaled_msg);
    	voxelized_pub.publish(voxelized_msg);
    	normal_pub.publish(normal_msg); /*
    	normal_variation_pub.publish(normal_variation_msg);
    	non_vegetation_pub.publish(non_vegetation_msg);
    	vegetation_pub.publish(vegetation_msg); */
    	dem_pub.publish(dem_msg);
    	dem_filtered_pub.publish(filtered_dem_msg);
    	dem_cluster_pub.publish(dem_cluster_msg);
    	reattached_dem_pub.publish(reattached_dem_msg);
    	dem_filled_pub.publish(dem_filled_msg);
    	dsm_pub.publish(dsm_msg);
    	chm_pub.publish(chm_msg);
    	vegetation_pub.publish(vegetation_msg);

    	dem_image_pub.publish(dem_image_msg);
		dsm_image_pub.publish(dsm_image_msg);
		dem_image_reattached_pub.publish(reattached_dem_image_msg);
		dem_image_filled_pub.publish(dem_filled_image_msg);

		//dem_smoothed_pub.publish(dem_smoothed_msg);

    	ros::Duration(2.0).sleep();
    } 
}