//=================================================================================================
// Copyright (c) 2013, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>


class CameraInfoPub
{
  public:
    CameraInfoPub()
    {
      ros::NodeHandle nh("");
      cam_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>("/multisense_sl/left/camera_info",1);
      image_sub_ = nh.subscribe("/multisense_sl/left/image_rect", 5, &CameraInfoPub::imageCallback, this);
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& img)
    {
      sensor_msgs::CameraInfo cam_info;

      cam_info.header = img->header;


      cam_info.height = 544;
      cam_info.width = 1024;

      cam_info.P[0] = 570.1205444335938;
      cam_info.P[2] = 519.0;
      cam_info.P[5] = 570.1205444335938;
      cam_info.P[6] = 267.5;
      cam_info.P[10] = 1.0;

      cam_info_pub_.publish(cam_info);
    }


  protected:
    ros::Publisher cam_info_pub_;
    ros::Subscriber image_sub_;


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flor_cam_info_pub_node");

  CameraInfoPub ci_pub;

  ros::spin();
}
