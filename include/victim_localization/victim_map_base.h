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
typedef geometry_msgs::PoseStamped PoseStamped;

using namespace grid_map;

struct detector_status {
  bool victim_found;
  Position victim_loc;
};


class Victim_Map_Base
{

protected://get it from config ..
  double Prob_D_H;  //P(D|H)
  double Prob_D_Hc;  //P(D|Hc)
  double Prob_Dc_H; //P(Dc|H)
  double Prob_Dc_Hc; //P(Dc|Hc)

public:
  std::string layer_name="general";//="victim";
  ros::NodeHandle nh_;
  ros::Publisher pub_map;
  ros::Subscriber sub_loc;
  ros::Publisher pub_polygon;


  float const_;
  Pose current_loc_;
  double current_yaw_;


  double HFOV_deg; //29 deg
  double max_depth_d;
  double x_arena_max;
  double y_arena_max;  //...
  grid_map::GridMap map;
  grid_map::Polygon polygon;



  //Detection_info//
  Point detect_loc_;
  bool is_detect_;
  Point p1; // first corner point for triangle
  Point p2; // second corner point for triangle



  //**************//


  virtual void Update(){};
  virtual detector_status getDetectionStatus();


  Position approximate_detect(Position x);
  //bool valid(Position loc);
  void publish_Map();
  grid_map::Polygon draw_FOV();
  void callbackdrawFOV(const PoseStamped &ps_stamped);

  std::string getlayer_name(void);
  void setlayer_name(std::string layer_);
  void setCurrentPose(Pose ps);
  void setDetectionResult(Point p, bool is_detect_);

  Victim_Map_Base();
  ~Victim_Map_Base();

};




#endif // VICTIM_MAP_H
