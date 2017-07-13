/*
Copyright 2015, Giacomo Dabisias"
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
Giacomo  Dabisias, PhD Student
PERCRO, (Laboratory of Perceptual Robotics)
Scuola Superiore Sant'Anna
via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy
*/

#include "k2g.h"
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>
#include <libfreenect2/libfreenect2.hpp>
// extra headers for writing out ply file
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#ifdef WITH_SERIALIZATION
#include "serialization.h"
#endif

const unsigned short MAX_DEPTH = 5000; // 5 m

// ./Kinect2Grabber_convert -processor 3
int main(int argc, char * argv[])
{
  std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] -processor options 0,1,2,3 correspond to CPU, OPENCL, OPENGL, CUDA respectively\n";
  std::cout << "Press \'x\' to store the calibrations." << std::endl;

  Processor freenectprocessor = OPENGL;

  if(argc > 1){
      freenectprocessor = static_cast<Processor>(atoi(argv[1]));
  }

  cv::Mat color, raw_depth, raw_ir;
  boost::shared_ptr<cv::Mat> color_ptr;
  K2G k2g(freenectprocessor);

  color = cv::imread("test/color.png");
  cv::cvtColor(color, color, CV_BGR2BGRA);
  // Read depth image
  std::string file_path = "test/depth.bin";
  FILE * fp_depth;
  fp_depth = std::fopen(file_path.c_str(), "rb");
  unsigned char *depthBuf[512*424*4];
  fread(depthBuf, 4,  512*424, fp_depth);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(512,424));

  // Build frames
  libfreenect2::Frame rgb(color.cols, color.rows, 4);
  libfreenect2::Frame depth(512, 424, 4);
  rgb.data = color.data;
  depth.data = (unsigned char*)depthBuf;

  k2g.getCloud(&rgb, &depth, cloud);

  pcl::PCDWriter writer_pcd;
  writer_pcd.write ("test/cloud.pcd", *cloud, true);


  k2g.shutDown();
  return 0;
}
