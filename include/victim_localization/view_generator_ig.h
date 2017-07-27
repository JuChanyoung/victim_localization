#ifndef VIEW_GENERATOR_IG_H
#define VIEW_GENERATOR_IG_H


#include <iostream>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <victim_localization/common.h>

typedef geometry_msgs::Pose Pose;

class view_generator_IG
{

protected:
   double res_x_, res_y_, res_z_, res_yaw_;

   double nav_bounds_x_max_, nav_bounds_y_max_, nav_bounds_z_max_;
   double nav_bounds_x_min_, nav_bounds_y_min_, nav_bounds_z_min_;

   double uav_fixed_height;
public:
   //....variables....
  double obj_bounds_x_max_, obj_bounds_y_max_, obj_bounds_z_max_;
  double obj_bounds_x_min_, obj_bounds_y_min_, obj_bounds_z_min_;
  Pose current_pose_;
  std::vector<Pose> generated_poses;

  // Visualizer
  int vis_marker_array_prev_size_;
  int vis_sphere_counter_;
  ros::Publisher pub_view_marker_array_;
  ros::Publisher pub_view_drone_marker_;

  //...methods...
  view_generator_IG();
  void setCurrentPose(Pose p);

  bool isInsideBounds(Pose p);
  bool isValidViewpoint(Pose p);

  //generate views
  void generateViews(); //viewpoints is also generated at current pose


 //visualize
  void visualize(std::vector<Pose> valid_poses, std::vector<Pose> invalid_poses);
  visualization_msgs::Marker visualizeDeleteArrowMarker(int id);
  visualization_msgs::Marker visualizeCreateArrowMarker(int id, Pose pose, bool valid, double max_z = 0, double min_z = 0);
  void visualizeDrawSphere(Pose p, double r);

};

#endif // VIEW_GENERATOR_IG_H
