/*
Copyright 2016, Giacomo Dabisias & Michele Mambrini"
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
@Author 
Giacomo  Dabisias, PhD Student & Michele Mambrini
PERCRO, (Laboratory of Perceptual Robotics)
Scuola Superiore Sant'Anna
via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy
*/
#include "ros_k2g_grabber.h"
// extra headers for writing out ply file
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/ply_io.h>

void
KeyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void * data)
{
	std::string pressed;
	pressed = event.getKeySym();
	PlySaver * s = (PlySaver*)data;
	if(event.keyDown ())
	{
		if(pressed == "s")
		{
		  
			pcl::PLYWriter writer;
			std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
			std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());
			writer.write ("cloud_" + now, *(s->cloud_), s->binary_, s->use_camera_);
			std::cout << "saved " << "cloud_" + now + ".ply" << std::endl;
		}
		if(pressed == "e")
		{
			s->K2G_ros_.setShutdown();
			std::cout << "SHUTTING DOWN" << std::endl;
		}
	}
}


int main(int argc, char *argv[])
{
	std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] -processor options 0,1,2,3 correspond to CPU, OPENCL, OPENGL, CUDA respectively\n";
	Processor freenectprocessor = OPENGL;

	if(argc > 1){
		freenectprocessor = static_cast<Processor>(atoi(argv[1]));
	}
	ros::init(argc, argv, "RosKinect2Grabber");

	Kinect2Grabber K2G_ros(freenectprocessor);
/*
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
	cloud = K2G_ros.getCloud();

	cloud->sensor_orientation_.w() = 0.0;
	cloud->sensor_orientation_.x() = 1.0;
	cloud->sensor_orientation_.y() = 0.0;
	cloud->sensor_orientation_.z() = 0.0; 

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

	PlySaver ps(cloud, false, false, K2G_ros);
	viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);	
*/
	while((ros::ok()) && (!K2G_ros.terminate()))
	{  		
		//viewer->spinOnce ();
		K2G_ros.publishAll();  
		//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		//viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");     	 
	}

	K2G_ros.shutDown();
	return 0;
}
