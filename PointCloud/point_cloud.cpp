#include "point_cloud.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
pcl::visualization::PCLVisualizer::Ptr visualizer = std::make_shared<pcl::visualization::PCLVisualizer>();
