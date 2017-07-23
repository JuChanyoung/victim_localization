#ifndef VICTIM_MAP_H
#define VICTIM_MAP_H

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include "math.h"
#include <iostream>
#include <string>
#include <vector>
#include "geometry_msgs/PointStamped.h"
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <sensor_msgs/PointCloud.h>
#include "victim_localization/common.h"

typedef geometry_msgs::Pose Pose;
typedef geometry_msgs::Point Point;

using namespace grid_map;

class Victim_Map_Base
{

protected://get it from config ..
  double Prob_D_H;  //P(D|H)
  double Prob_D_Hc;  //P(D|Hc)
  double Prob_Dc_H; //P(Dc|H)
  double Prob_Dc_Hc; //P(Dc|Hc)
  double HFOV_deg; //29 deg
  double max_depth_d;

  double x_arena_max;
  double y_arena_max;  //...
  grid_map::GridMap map;
  grid_map::Polygon polygon;

public:
  std::string layer_name="victim";
  ros::NodeHandle nh_;
  ros::Publisher pub_map;
  ros::Publisher pub_polygon;
  ros::Subscriber sub_map;
  float const_;
  Pose current_loc_;

  //Detection_info//
  Pose detect_loc_;
  bool is_detect_;
  Point p1; // first corner point for triangle
  Point p2; // second corner point for triangle

  //**************//


  virtual void Update(Pose c,Pose d,bool is_detect);
  Position approximate_detect(Position x);
  //bool valid(Position loc);
  void publish_Map();
  void draw_FOV();

  std::string getlayer_name(void);
  void setlayer_name(std::string layer_);

  Victim_Map_Base();
  ~Victim_Map_Base();

};




#endif // VICTIM_MAP_H
