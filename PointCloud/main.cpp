#include <thread>
#include <chrono>
#include <conio.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "filters.h"
#include "init.h"
#include "point_cloud.h"


using namespace std::chrono_literals;

void listen_for_key_pressed()
{
	if (_kbhit())
	{
		int ch = _getch();
		switch (ch)
		{
		case 67:
			//hit C
		case 99:
			//hit c
			std::cout << "detected c pressed" << std::endl;
			reset_camera();
			break;
		case 80:
			//hit P
			std::cout << "detected P pressed" << std::endl;
		case 112:
			std::cout << "detected p pressed" << std::endl;
			//hit p
			pass_through_filter();
			break;
		case 82:
			//hit R
			std::cout << "detected R pressed" << std::endl;
		case 114:
			//hit r
			std::cout << "detected r pressed" << std::endl;
			radius_outlier_removal_filter();
			break;
		case 83:
			//hit S
			std::cout << "detected S pressed" << std::endl;
		case 115:
			//hit s
			std::cout << "detected s pressed" << std::endl;
			statical_outlier_removal_filter();
			break;
		case 73:
			//hit I
			std::cout << "detected I pressed" << std::endl;
		case 105:
			//hit i
			std::cout << "detected i pressed" << std::endl;
			reset_point_cloud();
			break;
		case 86:
			//hit V
		case 118:
			//hit v
			std::cout << "detected v pressed" << std::endl;
			voxel_grid_filter();
			break;
		default:
			break;
		}
	}
}

void render()
{
	if (visualizer)
	{
		while (!visualizer->wasStopped())
		{
			visualizer->spinOnce(100);
			listen_for_key_pressed();
			std::this_thread::sleep_for(20ms);
		}
	}
}


void run()
{
	init();
	render();
}

int main()
{
	run();
	return 0;
}
