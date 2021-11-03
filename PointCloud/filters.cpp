#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "point_cloud.h"
#include "filters.h"

void pass_through_filter()
{
	pcl::PassThrough<pcl::PointXYZ> pass_through_filter(false);
	pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
	pass_through_filter.setInputCloud(cloud);
	pass_through_filter.setFilterFieldName("x");
	pass_through_filter.setKeepOrganized(false);
	pass_through_filter.setFilterLimits(-0.750f, 0.465f);
	pass_through_filter.filter(filtered_cloud);
	std::cout << "filtered point by pass through filter!" << std::endl;
	visualizer->updatePointCloud(filtered_cloud.makeShared());
}

void statical_outlier_removal_filter()
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter(false);
	pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
	sor_filter.setMeanK(8);
	sor_filter.setStddevMulThresh(1.);
	sor_filter.setKeepOrganized(false);
	sor_filter.setInputCloud(cloud);
	sor_filter.filter(filtered_cloud);
	std::cout << "filtered point by statical outlier removal filter!" << std::endl;
	visualizer->updatePointCloud(filtered_cloud.makeShared());
}


void radius_outlier_removal_filter()
{
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror_filter(false);
	pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
	ror_filter.setRadiusSearch(0.005f);
	ror_filter.setMinNeighborsInRadius(2);
	ror_filter.setInputCloud(cloud);
	ror_filter.setKeepOrganized(false);
	ror_filter.filter(filtered_cloud);
	std::cout << "filtered point by radius outlier removal filter!" << std::endl;
	visualizer->updatePointCloud(filtered_cloud.makeShared());
}

void voxel_grid_filter()
{
	pcl::VoxelGrid<pcl::PointXYZ> vg_filter;
	pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
	vg_filter.setInputCloud(cloud);
	vg_filter.setLeafSize(0.005,0.005,0.005);
	vg_filter.filter(filtered_cloud);
	std::cout << "filtered point by voxel grid filter!" << std::endl;
	visualizer->updatePointCloud(filtered_cloud.makeShared());
}
