
/* **** Overview of Method

1) Aggregate neighborhoods of points
2) For each pair of points, find the point closest to their normal axes
	a) skip pairs with parallel normals
	b) take cross product of the axes (cylinder axis C)
	c) project distance between points onto cylinder tangent (magnitude of C cross total distance vector divided by magnitude of C)
	d) find cylinder radius based on angle difference between normals and above distance: r = d*sin(90-alpha/2)/sin(alpha)
	e) find midpoint of the axis segment between these points: (S + n_s*r + T + n_t*r)/2
3) Line RANSAC across all the points found as above for each pair
4) Point pairs kept are concentric about this axis! But also finds cones? or parabolic cones... or other weird conics

*/



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

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

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

int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "lidar_testing");

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    	ros::console::notifyLoggerLevelsChanged();

  	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  	// Input Data
  	pcl::PointCloud<pcl::PointNormal>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointNormal>());




	pcl::PointCloud<pcl::PointNormal>::Ptr axis_candidates(new pcl::PointCloud<pcl::PointNormal>());

	pcl::KdTreeFLANN<pcl::PointNormal> tree;
	tree.setInputCloud(scene_cloud);
	std::vector<int> nearest_indices;
	std::vector<float> nearest_dist_squareds;

	for(int i=0; i<scene_cloud->points.size(); i++)
	{
		pcl::PointNormal source_point = scene_cloud->points[i];
		if(tree.nearestKSearch(source_point, 30, nearest_indices, nearest_dist_squareds))
		{
			for(int j=0; j<nearest_indices.size(); j++)
			{
				pcl::PointNormal target_point = scene_cloud->points[nearest_indices[j]];

				if(source_point.normal_x == target_point.normal_x && 
				   source_point.normal_y == target_point.normal_y &&
				   source_point.normal_z == target_point.normal_z)
					continue; 			// Skip pairs of points with parallel normals

				Eigen::Vector3f cross_product, point_distance, secant_partial, source_point_eig, target_point_eig;
				cross_product << source_point.normal_y*target_point.normal_z - source_point.normal_z*target_point.normal_y, 
								 source_point.normal_z*target_point.normal_x - source_point.normal_x*target_point.normal_z, 
								 source_point.normal_x*target_point.normal_y - source_point.normal_y*target_point.normal_x; // get cross product of the two normals
				point_distance << source_point.x-target_point.x, source_point.y-target_point.y, source_point.z-target_point.z;
				secant_partial = cross_product.cross(point_distance);
				float secant_distance = sqrt(secant_partial.dot(secant_partial))/sqrt(cross_product.dot(cross_product));
				float angle_offset = asin(sqrt(cross_product.dot(cross_product)));
				float cylinder_radius = secant_distance * sin(90-angle_offset) / sin(angle_offset); // this can definitely be simplified with trig...
				source_point_eig << source_point.x, source_point.y, source_point.z;
				target_point_eig << target_point.x, target_point.y, target_point.z;
				pcl::PointNormal midpoint;
				midpoint.x = (source_point_eig(0) + target_point_eig(0) + source_point.normal_x*cylinder_radius + target_point.normal_x*cylinder_radius) / 2;
				midpoint.y = (source_point_eig(1) + target_point_eig(1) + source_point.normal_y*cylinder_radius + target_point.normal_y*cylinder_radius) / 2;
				midpoint.z = (source_point_eig(2) + target_point_eig(2) + source_point.normal_z*cylinder_radius + target_point.normal_z*cylinder_radius) / 2;
				// Do something with the radius?
				axis_candidates->points.push_back(midpoint);
			}
			// line ransac on axis candidates
			if(true) // ransac_is_good
				break;
		}
		else 
			ROS_ERROR_STREAM("Warning... failed to find neighbors of point " << i << "with coordinates " << scene_cloud->points[i].x << " " << scene_cloud->points[i].x << " " << scene_cloud->points[i].z);

		// region growing
	}

}