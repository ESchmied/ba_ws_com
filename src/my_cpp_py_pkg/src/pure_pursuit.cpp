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
#include <tuple>
#include <cmath>
#include <iostream>
#include <fstream>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std;
//using namespace std::chrono_literals;
using std::placeholders::_1;

const float LOOKAHEAD_DISTANCE = 1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class Pure_Pursuit_Node : public rclcpp::Node
{
public:
  Pure_Pursuit_Node()
  : Node("Pure_Pursuit")
  {
    marker_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("csv_point", 10);
    goal_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("current_goal_point", 10);
    drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("ego_racecar/odom",10, std::bind(&Pure_Pursuit_Node::odom_callback, this) );
    
    last_visited_waypoints.clear();

    string file_name = "/home/emelie/ba_ws/src/ba_roslab/ba_roslab/map/Spielberg_map_klein_race_line.csv";
    ifstream Raceline_CSV;
    Raceline_CSV.open(file_name);

    //import waypoints aus csv file zur not marker in WayPOints
    string line;
    vector<String>
  
    while (getline(Raceline_CSV, line)){
//Todo
    }


  }

    //sichergehen das der Vector zu beginn leer ist
    std::vector<float> last_visited_waypoints;




private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

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
    for (p : this->path_points){
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

  void odom_callback(const nav_msgs::msg::Odometry msg){
    tuple<float, float, float> car_position;
    tuple<double, double, double> car_orientation;
    car_position = make_tuple(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.x);
    car_orientation = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w );
  }
  
};

tuple<double,double,double> euler_from_quaternion(float x, float y, float z, float w){
  tuple<double, double, double> euler_angles;
  tf2::Quaternion q;
  q.setValue(x,y,z,w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  euler_angles= make_tuple(roll, pitch, yaw);
  return euler_angles;
}

float calc_euc_dist(tuple<float,float> point1, tuple<float,float> point2){
  float distance_x = get<0>(point1)-get<0>(point2);
  float distance_y= get<1>(point1)-get<1>(point2);
  float distance = sqrt(distance_x*distance_x + distance_y*distance_y);
  return distance;
}

//not sure what its used for
float calc_turning_angle(float curvature){
  return curvature;
}

int find_nearest_waypoint(tuple<float, float>){
  float smallest_dist = INFINITY;

  for(){
    //todo 
  }
}

int find_next_waypoint(int current_waypoint, tuple<float, float> car_position){
  int new_waypoint;
  if(calc_euc_dist(this->path_points[current_waypoint], car_position)< LOOKAHEAD_DISTANCE){
    new_waypoint = current_waypoint + 1;
  }
  else{
    return current_waypoint;
  }

  if(new_waypoint == size(this->path_points)){
    return 0;
  } 

  return new_waypoint;  
}

tuple<float, float, float> transform_waypoint(tuple<float, float, float> waypoint, tuple<float, float, float> car_pos, tuple<float, float, float> car_orientation){
  tuple<float, float, float> waypoints;
  float theta = get<2>(car_orientation);

  get<0>(waypoint) = get<0>(waypoint) - get<0>(car_pos);
  get<1>(waypoint) = get<1>(waypoint) - get<1>(car_pos);

  get<0>(waypoint) = get<0>(waypoint)*cos(theta) - get<1>(waypoint)*sin(theta);
  get<1>(waypoint) = -get<0>(waypoint)*sin(theta) + get<1>(waypoint)*cos(theta);

  waypoints = waypoint;
  return waypoints;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pure_Pursuit_Node>());
  rclcpp::shutdown();
  return 0;
}
