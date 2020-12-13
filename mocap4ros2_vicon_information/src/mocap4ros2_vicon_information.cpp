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


class Mocap4ros2ViconInformation : public rclcpp::Node
{
public:
  Mocap4ros2ViconInformation()
  : Node("mocap4ros2_vicon_information")
  {
      marker_sub_ = this->create_subscription<mocap4ros2_msgs::msg::Markers>("/vicon/markers", 10, std::bind(&Mocap4ros2ViconInformation::callback, this, _1));
  }
  
  void callback(const mocap4ros2_msgs::msg::Markers::SharedPtr msg) const
  {
    RCLCPP_WARN(this->get_logger(), "MARKER: [%s, %d]. At time [sec: %d,  nanosec: %d]", msg->header.frame_id, msg->frame_number, msg->header.stamp.sec,msg->header.stamp.nanosec);

    for(unsigned int i = 0; i <= msg->markers.size(); i++){
        RCLCPP_INFO(this->get_logger(), "Translation [x: %d,  y: %d,  z: %d]", msg->markers[i].translation.x,  msg->markers[i].translation.y,  msg->markers[i].translation.z);

    }
    RCLCPP_WARN(this->get_logger(), "---------------------------------------------------------------------------------------------------");
  }


  void step()
  {
    
  }

protected:

    rclcpp::Subscription<mocap4ros2_msgs::msg::Markers>::SharedPtr marker_sub_;
    int count;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Mocap4ros2ViconInformation>();

  rclcpp::Rate loop_rate(10); 
  while (rclcpp::ok()) {
    node->step();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  
  rclcpp::shutdown();

  return 0;
}