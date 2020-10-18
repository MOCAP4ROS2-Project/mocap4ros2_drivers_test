// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Lorena Bajo Rebollo <lorena.bajo@urjc.es>


#include <math.h>
#include <iostream> 
#include <memory>
#include <string>
#include <map>

#include <stdlib.h>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "mocap4ros2_msgs/msg/marker.hpp"
#include "mocap4ros2_msgs/msg/markers.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer_interface.h>

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/create_timer_ros.h"

#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;


class Mocap4ros2ViconApp : public rclcpp::Node
{
public:
  Mocap4ros2ViconApp()
  : Node("mocap4ros2_vicon_app"), buffer_(this->get_clock())
  {
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
      marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("visualization_maker", 0);
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

      count =0;

  }
  
  void process_tf(geometry_msgs::msg::TransformStamped tf_, geometry_msgs::msg::TransformStamped tf_old_)
  {

    if (tf_.transform.translation != tf_old_.transform.translation || tf_.transform.rotation != tf_old_.transform.rotation){

      visualization_msgs::msg::Marker msg;

      geometry_msgs::msg::TransformStamped tf_aux_;

      tf_aux_.transform.translation.x = tf_.transform.translation.x - tf_old_.transform.translation.x;
      tf_aux_.transform.translation.y = tf_.transform.translation.y - tf_old_.transform.translation.y;
      tf_aux_.transform.translation.z = tf_.transform.translation.z - tf_old_.transform.translation.z;


      tf2::Quaternion qt;
      qt.setRPY(0.0, 0.0, 0.1);
      qt.normalize();

      tf_aux_.transform.rotation = tf2::toMsg(qt);

      msg.header.frame_id = "/my_tf";
      msg.header.stamp = now();
      msg.ns = "markers";
      msg.id = 0;
      msg.type = visualization_msgs::msg::Marker::ARROW;
      msg.action = visualization_msgs::msg::Marker::ADD;
      msg.frame_locked = false;

      msg.pose.position.x = tf_aux_.transform.translation.x;
      msg.pose.position.y = tf_aux_.transform.translation.y;
      msg.pose.position.z = tf_aux_.transform.translation.z;
      msg.pose.orientation.x = tf_aux_.transform.rotation.x ;
      msg.pose.orientation.y = tf_aux_.transform.rotation.y;
      msg.pose.orientation.z = tf_aux_.transform.rotation.z;
      msg.pose.orientation.w = 1.0 ;

      msg.scale.x = 1;
      msg.scale.y = 0.1;
      msg.scale.z = 0.1;
      msg.color.a = 1.0;
      msg.color.r = 0.0;
      msg.color.g = 1.0;
      msg.color.b = 0.0;

      //builtin_interfaces::msg::Duration dur;
      //dur.sec = 10;
      //msg.lifetime = dur;
      marker_pub_->publish(msg);

      // tf_aux_.header.frame_id = "map"; //map
      // tf_aux_.child_frame_id = "my_tf_aux";
      // tf_aux_.header.stamp = now();

      // tf_broadcaster_->sendTransform(tf_aux_);

    }

  }


  void step()
  {
    rclcpp::Time rclcpp_time = now();
    tf2::TimePoint tf2_time(std::chrono::nanoseconds(rclcpp_time.nanoseconds()));

    std::string target_frame_ = "map";
    std::string source_frame_= "my_tf";

    try
      {
        geometry_msgs::msg::TransformStamped tf_;
        tf_ = buffer_.lookupTransform(target_frame_, source_frame_, tf2_time);

        RCLCPP_WARN(this->get_logger(), "TF [%s -> %s]. At time [sec: %d,  nanosec: %d]",target_frame_.c_str(), source_frame_.c_str(), tf_.header.stamp.sec,tf_.header.stamp.nanosec);

        auto translation = tf_.transform.translation;
        auto rotation = tf_.transform.rotation;
        RCLCPP_INFO(this->get_logger(), "Translation [x: %d,  y: %d,  z: %d]", translation.x, translation.y, translation.z);
        RCLCPP_INFO(this->get_logger(), "Rotation [x: %d,  y: %d,  z: %d,  w: %d]", rotation.x, rotation.y, rotation.z, rotation.w);

        process_tf(tf_, tf_old_);
        
        tf_old_ = tf_;
        
    
      }catch(tf2::TransformException& ex){

        RCLCPP_ERROR(this->get_logger(), "The frame doesn't exists");
      }
  }


protected:

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    tf2_ros::Buffer buffer_;
    geometry_msgs::msg::TransformStamped tf_old_;
    int count;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Mocap4ros2ViconApp>();

  rclcpp::Rate loop_rate(10); 
  while (rclcpp::ok()) {
    node->step();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  
  rclcpp::shutdown();

  return 0;
}