#pragma once
#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
extern pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
extern pcl::visualization::PCLVisualizer::Ptr visualizer;



#endif
