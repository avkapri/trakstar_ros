/*********************************************************************
*
* This is free and unencumbered software released into the public domain.
* 
* Anyone is free to copy, modify, publish, use, compile, sell, or
* distribute this software, either in source code form or as a compiled
* binary, for any purpose, commercial or non-commercial, and by any
* means.
* 
* In jurisdictions that recognize copyright laws, the author or authors
* of this software dedicate any and all copyright interest in the
* software to the public domain. We make this dedication for the benefit
* of the public at large and to the detriment of our heirs and
* successors. We intend this dedication to be an overt act of
* relinquishment in perpetuity of all present and future rights to this
* software under copyright law.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
* 
* For more information, please refer to <http://unlicense.org/>
* 
**********************************************************************/

/** 
 * Author: Sean Seungkook Yun <seungkook.yun@sri.com> 
*/

#include <string>
#include <ros/ros.h>
#include "trakstar/PointATC3DG.hpp"
#include "trakstar/TrakstarMsg.h"
#include "tf/tf.h"

// Visualization
#include <tf/transform_broadcaster.h>

//writing to file
#include <fstream>


//for shared_ptr
#include <boost/shared_ptr.hpp>

using namespace trakstar;
using std::string;

bool recording = true;


struct DataPoint {
    tf::Vector3 pos;
    tf::Quaternion q;
};

void stop_recording(const ros::TimerEvent&)
{
    ROS_INFO("stop recording callback triggered");
    recording = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trakstar_driver");
  ros::NodeHandle n, n_private("~");
  ros::Time::init();

  bool hemisphere_back;
  n_private.param<bool>("hemisphere_back", hemisphere_back, false);

  //initialize hardware
  ROS_INFO("Initializing TRAKSTAR. Please wait....");
  PointATC3DG bird_;
  if( !bird_ ) {
    ROS_ERROR("can't open trakstar"); 
    return -1;
  }

  int frequency = 255;
  n_private.getParam("frequency", frequency);
  ROS_INFO("Frequency: %d", frequency);

  bird_.setMeasurementRate(static_cast<float>(frequency));

  ROS_INFO("Initialization Complete.");

  bird_.setSuddenOutputChangeLock( 0 );	
  int num_sen=bird_.getNumberOfSensors();
  ROS_INFO("Number of trakers: %d", num_sen);


  if(num_sen != 1)
  {
      ROS_ERROR("exactly one tracker should be connected");
      return -1;
  }


  ROS_INFO("Output is set: position/quaternion");
  for (int i=0; i<num_sen; i++) {
    bird_.setSensorQuaternion(i);
    if (hemisphere_back)
      bird_.setSensorHemisphere(i, HEMISPHERE_REAR);
    else 
      bird_.setSensorHemisphere(i, HEMISPHERE_FRONT);
  }

  bool range_72inch;
  n_private.param<bool>("range_72inch", range_72inch, false);
  if (range_72inch)
    bird_.setMaximumRange(true);

  double dX, dY, dZ;
  double* quat=new double[4];


  bool publish_tf;
  n_private.param<bool>("publish_tf", publish_tf, false);
  if(publish_tf)
    ROS_INFO("Publishing frame data to TF.");

  //offset of how the sensor tip is attached
  double px, py, pz;
  n_private.param<double>("pivot_x", px, 0);
  n_private.param<double>("pivot_y", py, 0);
  n_private.param<double>("pivot_z", pz, 0);
  tf::Vector3 trakstar_attach_pos(px, py, pz);

  //orientation of how the sensor tip is attached
  double rx, ry, rz;
  n_private.param<double>("attach_roll", rx, 0);
  n_private.param<double>("attach_pitch", ry, 0);
  n_private.param<double>("attach_yaw", rz, 0);
  tf::Matrix3x3 trakstar_attach;
  trakstar_attach.setEulerYPR(rz, ry, rx);

  //offset of how the sensor tip is attached
  double px1, py1, pz1;
  n_private.param<double>("pivot_x1", px1, 0);
  n_private.param<double>("pivot_y1", py1, 0);
  n_private.param<double>("pivot_z1", pz1, 0);
  tf::Vector3 trakstar_attach_pos1(px1, py1, pz1);

  //orientation of how the sensor tip is attached
  double rx1, ry1, rz1;
  n_private.param<double>("attach_roll1", rx1, 0);
  n_private.param<double>("attach_pitch1", ry1, 0);
  n_private.param<double>("attach_yaw1", rz1, 0);
  tf::Matrix3x3 trakstar_attach1;
  trakstar_attach1.setEulerYPR(rz1, ry1, rx1);

// Is this needed as individual application?
//  // Initialize ROS stuff
//  ros::Publisher trakstar_pub = n.advertise<trakstar::TrakstarMsg>("trakstar_msg", 1);
//  ros::Publisher trakstar_raw_pub = n.advertise<trakstar::TrakstarMsg>("trakstar_raw_msg", 1);
//  tf::TransformBroadcaster *broadcaster = 0;
//  if(publish_tf)
//    broadcaster = new tf::TransformBroadcaster();

  // mangle the reported pose into the ROS frame conventions
  const tf::Matrix3x3 ros_to_trakstar( -1,  0,  0,
	                   	      0,  1,  0,
	                   	      0,  0, -1 );
  ros::Rate loop_rate(frequency);

  string raw_file;
  ROS_INFO("Please hold the UID steady and rotate around the center");
  ROS_INFO("type \"y\" to start recording data");
  std::cin >> raw_file;

  std::vector<boost::shared_ptr<DataPoint> > recorded_values;

  std::cout << "3" << std::endl;
  sleep(1);
  std::cout << "2" << std::endl;
  sleep(1);
  std::cout << "1" << std::endl;
  sleep(1);
  std::cout << "RECORDING..." << std::endl;



  ros::Timer timer = n.createTimer(ros::Duration(10), stop_recording);

  std::ofstream myfile("/home/devuser/anette/tests/trakstar_test/recorded_data.txt", std::ofstream::out);

  while (recording)
  {
    std::vector<geometry_msgs::TransformStamped> transforms(num_sen);

    for( int i = 0; i <num_sen  ; ++i ) 
    {
	bird_.getCoordinatesQuaternion(i, dX, dY, dZ, quat);
	tf::Vector3 pos(dX, dY, dZ);
	tf::Quaternion q(-quat[1], -quat[2], -quat[3], quat[0]);
	tf::Matrix3x3 mat(q);

	mat=ros_to_trakstar*mat;

	if (i<1) {
	  mat*=trakstar_attach;
	  pos=ros_to_trakstar*pos+ mat*trakstar_attach_pos; 
        }
        else {
	  mat*=trakstar_attach1;
	  pos=ros_to_trakstar*pos+ mat*trakstar_attach_pos1; 
        }

    boost::shared_ptr<DataPoint> p = boost::make_shared<DataPoint>();
    p->pos = pos;
    p->q = q;
    recorded_values.push_back(p);

std::cout << pos.x() << " " << pos.y() << " " << pos.z() << " "  <<
              q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;

    myfile << pos.x() << " " << pos.y() << " " << pos.z() << " "  <<
              q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";

    }
    loop_rate.sleep();
    ros::spinOnce();
  }

  myfile.close();



  delete [] quat;
}









