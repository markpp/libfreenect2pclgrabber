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
#include <chrono>
// extra headers for writing out ply file
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

// ./Kinect2Grabber_raw -processor 3
int main(int argc, char * argv[])
{
  std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] -processor options 0,1,2,3 correspond to CPU, OPENCL, OPENGL, CUDA respectively\n";
  std::cout << "Press \'x\' to store the calibrations." << std::endl;

  Processor freenectprocessor = OPENGL;

  if(argc > 1){
      freenectprocessor = static_cast<Processor>(atoi(argv[1]));
  }

  namedWindow("Capturing", cv::WINDOW_AUTOSIZE);// Create a window for display.

  cv::Mat color, raw_depth, raw_ir;
  boost::shared_ptr<cv::Mat> color_ptr;
  K2G k2g(freenectprocessor);

  while(1)
  {
    std::chrono::high_resolution_clock::time_point tnow = std::chrono::high_resolution_clock::now();

    k2g.store_raw(true);

    std::chrono::high_resolution_clock::time_point tpost = std::chrono::high_resolution_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::duration<double>>(tpost-tnow).count() << std::endl;

    int key = cv::waitKey(20);
    if (key=='q')
    {
      break;
    }
  }
  k2g.shutDown();
  return 0;
}
