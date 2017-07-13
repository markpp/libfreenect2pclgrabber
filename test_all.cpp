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

struct PlySaver{

  PlySaver(cv::Mat *color, cv::Mat *raw_depth, cv::Mat *raw_ir, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud, bool binary, bool use_camera, K2G & k2g):
           color_(color), raw_depth_(raw_depth), raw_ir_(raw_ir), cloud_(cloud), binary_(binary), use_camera_(use_camera), k2g_(k2g){}

  cv::Mat *color_, *raw_depth_, *raw_ir_;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_;

  bool binary_;
  bool use_camera_;
  K2G & k2g_;
};

void displayFPS(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_, double delta)
{
  const int dx = 5;
  const int dy = 14;
  const int fs = 10;

  std::ostringstream oss;
  oss << "FPS: " << 1000.0/delta;
  std::string fps = oss.str();

  if (!viewer_->updateText (fps, dx, dy + (fs + 2), fs, 1.0, 1.0, 1.0, "fps"))
  {
    viewer_->addText (fps, dx, dy + (fs + 2), fs, 1.0, 1.0, 1.0, "fps");
  }
}

void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void * data)
{
  std::string pressed = event.getKeySym();
  PlySaver * s = (PlySaver*)data;
  if(event.keyDown ())
  {
    if(pressed == "s")
    {
      pcl::PLYWriter writer_ply;
      pcl::PCDWriter writer_pcd;
      std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
      std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());
      writer_pcd.write ("out/pc_pcd/cloud_" + now + ".pcd", *(s->cloud_), s->use_camera_);
      writer_ply.write("out/pc_ply/cloud_" + now + ".ply", *(s->cloud_), s->binary_, s->use_camera_);
      cv::imwrite("out/hd/color_" + now + ".png", *(s->color_));

      ofstream ir_out;
      ir_out.open("out/raw_ir/ir_" + now + ".dat");
      for (int i = 0; i < s->raw_ir_->rows; ++i) {
        unsigned short* rowPtr = (unsigned short*)s->raw_ir_->ptr(i);
        for (int j = 0; j < s->raw_ir_->cols; ++j)
          ir_out << rowPtr[j] << " ";
        ir_out << endl;
      }
      ir_out.close();

      cv::Mat ir;
      s->raw_ir_->convertTo(ir, CV_8U, 255.0 / 65000);
      cv::imwrite("out/ir/ir_" + now + ".png", ir);

      ofstream depth_out;
      depth_out.open("out/raw_depth/depth_" + now + ".dat");
      for (int i = 0; i < s->raw_depth_->rows; ++i) {
        unsigned short* rowPtr = (unsigned short*)s->raw_depth_->ptr(i);
        for (int j = 0; j < s->raw_depth_->cols; ++j)
          depth_out << rowPtr[j] << " ";
        depth_out << endl;
      }
      depth_out.close();

      cv::Mat depth;
      s->raw_depth_->convertTo(depth, CV_8U, 255.0 / MAX_DEPTH);
      cv::imwrite("out/depth/depth_" + now + ".png", depth);

      std::cout << "!!saved " << "data " + now + "!!" << std::endl;
      std::cout << "==========================" << std::endl;

    }
    if(pressed == "m")
    {
      s->k2g_.mirror();
    }
#ifdef WITH_SERIALIZATION
    if(pressed == "z")
    {
      if(!(s->k2g_.serialize_status())){
        std::cout << "serialization enabled" << std::endl;
        s->k2g_.enableSerialization();
      }
      else{
        std::cout << "serialization disabled" << std::endl;
        s->k2g_.disableSerialization();
      }
    }
#endif
    if(pressed == "x")
    {
        s->k2g_.storeParameters();
        std::cout << "stored calibration parameters" << std::endl;
    }
  }
}

// ./Kinect2Grabber_pc -processor 3
int main(int argc, char * argv[])
{
  std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] -processor options 0,1,2,3 correspond to CPU, OPENCL, OPENGL, CUDA respectively\n";
  std::cout << "Press \'s\' to store a cloud" << std::endl;
  std::cout << "Press \'x\' to store the calibrations." << std::endl;
#ifdef WITH_SERIALIZATION
  std::cout << "Press \'z\' to start/stop serialization." << std::endl;
#endif
  Processor freenectprocessor = OPENGL;

  if(argc > 1){
      freenectprocessor = static_cast<Processor>(atoi(argv[1]));
  }

  cv::Mat color, raw_depth, raw_ir;
  boost::shared_ptr<cv::Mat> color_ptr;
  K2G k2g(freenectprocessor);

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
  std::cout << "getting cloud" << std::endl;
  cloud = k2g.getCloud();

  k2g.printParameters();

  k2g.disableLog();

  cloud->sensor_orientation_.w() = 0.0;
  cloud->sensor_orientation_.x() = 1.0;
  cloud->sensor_orientation_.y() = 0.0;
  cloud->sensor_orientation_.z() = 0.0;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

  PlySaver ps(&color, &raw_depth, &raw_ir, cloud, false, false, k2g);
  viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);

  while(!viewer->wasStopped()){

    std::chrono::high_resolution_clock::time_point tnow = std::chrono::high_resolution_clock::now();

    viewer->spinOnce ();

    //k2g.get(color, raw_depth, cloud, true);
    //k2g.getIr(raw_ir);

    k2g.get(color, raw_depth, cloud, true);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

    viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");

    std::chrono::high_resolution_clock::time_point tpost = std::chrono::high_resolution_clock::now();
    //std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<double>>(tpost-tnow).count() * 1000 << std::endl;
    displayFPS(viewer, std::chrono::duration_cast<std::chrono::duration<double>>(tpost-tnow).count() * 1000);
  }


  k2g.shutDown();
  return 0;
}
