#define NOMINMAX // because the conflicts between pdal and pcl , this definition is necessary
#include <iostream>
#include <cstring>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasHeader.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <boost/filesystem.hpp>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
	char* las_file_name;
	if (argc>=3)
	{
		int filename_index = -1;
		for (int i = 0; i < argc; i++)
		{
			if (std::strcmp(argv[i],"-f")==0 && i + 1<argc)
			{
				filename_index = i + 1;
			}
		}
		if (filename_index==-1)
		{
			std::cerr << "ERROR: cannot find -f argument!" << std::endl;
			return -1;
		}
		else {
			las_file_name = argv[filename_index];
		}
	}
	else {
		las_file_name = new char[] {"./data/epsg_4326.las"};
	}
	
	boost::filesystem::path path(las_file_name);
	boost::system::error_code error;
	auto filestatus= boost::filesystem::status(path, error);
	if (error)
	{
		std::cerr << "ERROR: specified path is not exist!" << std::endl;
		return -1;
	}

	if (!boost::filesystem::exists(filestatus))
	{
		std::cerr << "ERROR: specified path is not exist!" << std::endl;
		return -1;
	}
	else if (!boost::filesystem::is_regular_file(filestatus))
	{
		std::cerr << "ERROR: file is not exist!" << std::endl;
		return -1;
	}
	else {
		std::cout << "INFO: data file is " << las_file_name << std::endl;
	}


	pdal::Options opt;
	pdal::LasReader las_reader;
	pdal::PointTable point_table;
	pdal::PointViewSet point_views;

	pcl::visualization::PCLVisualizer visualizer;


	visualizer.setWindowName("test02:read las");
	visualizer.initCameraParameters();
	opt.add("filename", las_file_name);
	las_reader.setOptions(opt);
	las_reader.prepare(point_table);
	std::cout << "INFO: loaded file " << las_reader.getName() << std::endl;

	std::cout << "    file name: " << las_reader.getName() << std::endl;
	std::cout << "    point numbers: " << las_reader.getNumPoints() << std::endl;
	std::cout << "    file signature: " << las_reader.header().fileSignature() << std::endl;
	std::cout << "    file source id: " << las_reader.header().fileSourceId() << std::endl;
	std::cout << "    project id: " << las_reader.header().projectId() << std::endl;
	std::cout << "    version: " << las_reader.header().versionString() << std::endl;
	std::cout << "    creation doy: " << las_reader.header().creationDOY() << std::endl;
	std::cout << "    creation year: " << las_reader.header().creationYear() << std::endl;
	std::cout << "    vlr offset: " << las_reader.header().vlrOffset() << std::endl;
	std::cout << "    point format: " << (int)las_reader.header().pointFormat() << std::endl;
	std::cout << "    compressed: " << (las_reader.header().compressed() ? "true" : "false") << std::endl;
	std::cout << "    max X: " << las_reader.header().maxX() << std::endl;
	std::cout << "    max Y: " << las_reader.header().maxY() << std::endl;
	std::cout << "    max Z: " << las_reader.header().maxZ() << std::endl;
	std::cout << "    min X: " << las_reader.header().minX() << std::endl;
	std::cout << "    min Y: " << las_reader.header().minY() << std::endl;
	std::cout << "    min Z: " << las_reader.header().minZ() << std::endl;

	point_views = las_reader.execute(point_table);

	std::cout << std::endl;


	


	for (auto& point_view : point_views)
	{
		pcl::PointCloud<pcl::PointXYZ> point_cloud;
		point_cloud.height = 1;
		point_cloud.width = point_view->pointSize();
		point_cloud.resize(point_view->pointSize());

		std::cout << "point view size: " << point_view->size() << std::endl;
		std::cout << std::endl;

		std::cout << "point dim type and name: " << std::endl;

		pdal::Dimension::IdList list = point_view->dims();
		for (auto value : list)
		{
			std::cout << defaultType(value) << "  " << name(value) << std::endl;
		}
		std::cout << std::endl;
		for (pdal::PointId i = 0; i < point_view->size(); ++i)
		{
			auto x = point_view->getFieldAs<float>(pdal::Dimension::Id::X, i);
			auto y = point_view->getFieldAs<float>(pdal::Dimension::Id::Y, i);
			auto z = point_view->getFieldAs<float>(pdal::Dimension::Id::Z, i);
			point_cloud.push_back(*(new pcl::PointXYZ(x, y, z)));
		}
		visualizer.addPointCloud<pcl::PointXYZ>(point_cloud.makeShared(), "cloud" + point_view->id());
	}

	pcl::visualization::Camera camera;
	camera.clip[0] = 0.105633, camera.clip[1] = 105.633;
	camera.focal[0] = -47.3417, camera.focal[1] = 15.5237, camera.focal[2] = 39.0595;
	camera.pos[0] = -47.3348, camera.pos[1] = 15.5218, camera.pos[0] = 39.0569;
	camera.view[0] = 0.292406, camera.view[1] = 0.949207, camera.view[2] = 0.116211;
	camera.window_size[0] = 960, camera.window_size[1] = 740;
	camera.window_pos[0] = 0, camera.window_pos[1] = 0, camera.window_pos[2] = 0;
	visualizer.setCameraParameters(camera);

	while (!visualizer.wasStopped())
	{
		
		visualizer.spinOnce(60);
		std::this_thread::sleep_for(15ms);
	}

	delete[] las_file_name;
	
	return 0;
}
