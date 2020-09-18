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

using namespace std::chrono_literals;
using std::placeholders::_1;


class Mocap4ros2CameraTest : public rclcpp::Node
{
public:
  Mocap4ros2CameraTest()
  : Node("mocap4ros2_camera_test")
  {
      tracked_frame_suffix_ = "vicon";
      test_num = 0;
      marker_pub_ = create_publisher<mocap4ros2_msgs::msg::Markers>(tracked_frame_suffix_+"/markers", 100);
  }
  
  int random_number(){

    int num;
    srand(time(NULL));
    
    for(int i = 1; i <= 10; i++)
    {
        num = 1 + rand() % (11 - 1);
    }

    return num;
  }

  void step()
  {

    mocap4ros2_msgs::msg::Markers markers_msg;
    markers_msg.header.stamp = now();
    markers_msg.frame_number = test_num++;

    mocap4ros2_msgs::msg::Marker new_marker;
    new_marker.translation.x = random_number();
    new_marker.translation.y = random_number();
    new_marker.translation.z = random_number();
    
    markers_msg.markers.push_back(new_marker);

    marker_pub_->publish(markers_msg);

  }


protected:

    rclcpp::Publisher<mocap4ros2_msgs::msg::Markers>::SharedPtr marker_pub_;
    int test_num;
    std::string tracked_frame_suffix_; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Mocap4ros2CameraTest>();

  rclcpp::Rate loop_rate(10); 
  while (rclcpp::ok()) {
    node->step();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  
  rclcpp::shutdown();

  return 0;
}