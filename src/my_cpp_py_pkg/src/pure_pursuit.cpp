// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class Pure_Pursuit_Node : public rclcpp::Node
{
public:
  Pure_Pursuit_Node()
  : Node("Pure_Pursuit"), count_(0)
  {
    marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("csv_point", 10);
    goal_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("current_goal_point", 10);


    timer_ = this->create_wall_timer(
      500ms, std::bind(&Pure_Pursuit_Node::timer_callback, this)); 
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;

  void publish_single_point(float x, float y, float z){
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "";
    marker.header.stamp = rclcpp::Clock().now();
    
    marker.ns= "basic_shapes";
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.id = 0;

    marker.scale.x =0.3;
    marker.scale.y =0.3;
    marker.scale.z =0.3;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    goal_marker_pub->publish(marker);
  }

  void publish_points(){
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;
    
    //array in vector for schleife und datentypen komisch
    for ( p : this->path_points){
      visualization_msgs::msg::Marker marker;
      
      marker.header.frame_id = "/map";
      marker.header.stamp = rclcpp::Clock().now();
      marker.type = 2;
      marker.id = marker_id,
      marker_id +=1;
      marker.scale.x =0.2; 
      marker.scale.y =0.2;
      marker.scale.z =0.2;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      marker.pose.position.x = p[0];
      marker.pose.position.y = p[1];
      marker.pose.position.z = p[2]*0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      //push_back adds element to the end of the vector
      marker_array.markers.push_back(marker);
    }
      
          
      marker_publisher->publish(marker_array);

  }

  void timer_callback(){
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pure_Pursuit_Node>());
  rclcpp::shutdown();
  return 0;
}
