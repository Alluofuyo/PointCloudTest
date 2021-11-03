#include "init.h"
#include <pcl/io/pcd_io.h>
#include "point_cloud.h"


void init()
{
	init_data();
	init_viewer();
}


void init_data()
{
	if (pcl::io::loadPCDFile(".\\data\\table_scene_lms400.pcd", *cloud) == -1)
	{
		PCL_ERROR("cannot import pcd file!");
		exit(-1);
	}
	std::cout << "loaded point size " << cloud->size() << std::endl;
}

void reset_camera()
{
	pcl::visualization::Camera camera;
	camera.clip[0] = 1.59393;
	camera.clip[1] = 4.29637;
	camera.focal[0] = -0.098315;
	camera.focal[1] = -0.079455;
	camera.focal[2] = -1.47315;
	camera.pos[0] = 0.00546734;
	camera.pos[1] = -0.56536;
	camera.pos[2] = 1.26911;
	camera.view[0] = 0;
	camera.view[1] = 0;
	camera.view[2] = 0;
	camera.window_pos[0] = 0;
	camera.window_pos[1] = 0;
	camera.window_size[0] = 1200;
	camera.window_size[1] = 720;
	visualizer->setCameraParameters(camera);
}

void init_viewer()
{
	visualizer->setWindowName("test02");
	visualizer->addPointCloud<pcl::PointXYZ>(cloud);
	visualizer->initCameraParameters();
	reset_camera();
	visualizer->registerPointPickingCallback([](const pcl::visualization::PointPickingEvent& event, void* viewer_void)
	{
		float x, y, z;
		event.getPoint(x, y, z);
		char t[512];
		char idx[512];
		sprintf_s(idx, "point%d", event.getPointIndex());
		sprintf_s(t, "selected point(%f,%f,%f)", x, y, z);
		std::cout << t << std::endl;
	});
}

void reset_point_cloud()
{
	std::cout << "reset point cloud!" << std::endl;
	visualizer->updatePointCloud(cloud);
}


