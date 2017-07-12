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

// ./Kinect2Grabber_vid -processor 3
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

  /*
  CV_FOURCC('P','I','M','1') = MPEG-1 codec
  CV_FOURCC('M','J','P','G') = motion-jpeg codec (does not work well)
  CV_FOURCC('M', 'P', '4', '2') = MPEG-4.2 codec
  CV_FOURCC('D', 'I', 'V', '3') = MPEG-4.3 codec
  CV_FOURCC('D', 'I', 'V', 'X') = MPEG-4 codec
  CV_FOURCC('U', '2', '6', '3') = H263 codec
  CV_FOURCC('I', '2', '6', '3') = H263I codec
  CV_FOURCC('F', 'L', 'V', '1') = FLV1 codec
  */
  cv::VideoWriter color_video, depth_video;
  color_video.open( "color_video.avi", CV_FOURCC('M','J','P','G'), 16, cv::Size (1920,1080), true);
  depth_video.open( "depth_video.avi", CV_FOURCC('M','P','4','2'), 16, cv::Size (512,424), false);

  while(1)
  {
    std::chrono::high_resolution_clock::time_point tnow = std::chrono::high_resolution_clock::now();

    k2g.get(color, raw_depth, true);

    std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
    std::string time_stamp = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());

    cv::Mat depth;
    raw_depth.convertTo(depth, CV_8U, 255.0 / MAX_DEPTH);


    color_video.write(color);
    depth_video.write(depth);

    std::chrono::high_resolution_clock::time_point tpost = std::chrono::high_resolution_clock::now();

    // Presentation
    std::ostringstream oss;
    oss << "FPS: " << 1.0/std::chrono::duration_cast<std::chrono::duration<double>>(tpost-tnow).count();
    std::string text = oss.str();
    int fontFace = cv::FONT_HERSHEY_PLAIN;
    double fontScale = 1;
    int thickness = 2;
    cv::Point textOrg(10, 50);
    //cv::putText(color, text, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness,8);
    //cv::imshow("color", color);
    cv::putText(depth, text, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness,8);
    cv::imshow("depth", depth);

    int key = cv::waitKey(30);
    if (key=='q')
    {
      color_video.release();
      depth_video.release();
      break;
    }
  }
  k2g.shutDown();
  return 0;
}
