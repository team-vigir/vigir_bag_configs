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

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


class AtlasPoseToWorldPub
{
  public:
    AtlasPoseToWorldPub()
    {
      ros::NodeHandle nh("");
      pelvis_pose_world_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/flor/state/pelvis_pose_world",1);

      pose_sub_ = nh.subscribe("/flor/controller/atlas_pose", 5, &AtlasPoseToWorldPub::poseCallback, this);

      odom_sub_ = nh.subscribe("/ground_truth_odom", 5, &AtlasPoseToWorldPub::odomCallback, this);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {

      tf::StampedTransform transform;

      transform.setOrigin(tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));

      tf::Quaternion orientation;
      tf::quaternionMsgToTF(msg->pose.orientation, orientation);

      transform.setRotation(orientation);

      transform.stamp_ = msg->header.stamp;
      transform.child_frame_id_ = "pelvis";
      transform.frame_id_ = "world";
      tfb_.sendTransform(transform);

    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {

      tf::StampedTransform transform;

      transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));

      tf::Quaternion orientation;
      tf::quaternionMsgToTF(msg->pose.pose.orientation, orientation);

      transform.setRotation(orientation);

      transform.stamp_ = msg->header.stamp;
      transform.child_frame_id_ = "pelvis";
      transform.frame_id_ = "world";
      tfb_.sendTransform(transform);

      geometry_msgs::PoseStamped pelvis_pose_msg;
      pelvis_pose_msg.pose = msg->pose.pose;
      pelvis_pose_msg.header.stamp = msg->header.stamp;
      pelvis_pose_msg.header.frame_id = "world";

      pelvis_pose_world_pub_.publish(pelvis_pose_msg);

    }


  protected:
    ros::Publisher pelvis_pose_world_pub_;

    tf::TransformBroadcaster tfb_;
    ros::Subscriber pose_sub_;
    ros::Subscriber odom_sub_;


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flor_atlas_to_world_pub_node");

  AtlasPoseToWorldPub atlas_to_world_pub;

  ros::spin();
}
