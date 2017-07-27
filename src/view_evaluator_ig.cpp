#include <iostream>

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include "victim_localization/view_evaluator_ig.h"
#include <victim_localization/view_generator_ig.h>
#include <victim_localization/victim_map_base.h>

view_evaluator_IG::view_evaluator_IG():
  info_selected_utility_(-std::numeric_limits<float>::infinity()) //-inf
{
  ros::param::param<double>("~HFOV_angle", HFOV_deg , 29);
  ros::param::param<double>("~maximum_depth_distance", max_depth_d , 4);
  ros::param::param<double>("~maximum_arena_width", x_arena_max , 20);
  ros::param::param<double>("~maximum_arena_height", y_arena_max , 20);

  const_=max_depth_d/cos(DEG2RAD(HFOV_deg));

}

double view_evaluator_IG::getCellEntropy(Index cell_)
{
  double p= mapping_module_->map.at(MapLayer,cell_);
  return - p*log(p) - (1-p)*log(1-p);
}

void view_evaluator_IG::setViewGenerator(view_generator_IG* v)
{
  view_gen_ = v;
}

void view_evaluator_IG::setMappingModule(Victim_Map_Base* m)
{
  mapping_module_ = m;
}

void view_evaluator_IG::update_parameters()
{
  MapLayer = mapping_module_->getlayer_name();
  current_pose_ = view_gen_->current_pose_;
  current_yaw_=pose_conversion::getYawFromQuaternion(current_pose_.orientation);
}

double view_evaluator_IG::calculateIG(Pose p){      //TOFIX project FOV from camera center not drone center
  Point p1,p2;
  p1.x= p.position.x+ max_depth_d*cos(current_yaw_) + const_* cos(current_yaw_ + 4.712);
  p2.x= p.position.x + max_depth_d*cos(current_yaw_) + const_* cos(current_yaw_ + 1.57);
  p1.y= p.position.y+ max_depth_d*sin(current_yaw_) + const_* sin(current_yaw_ + 4.712);
  p2.y= p.position.y + max_depth_d*sin(current_yaw_) + const_* sin(current_yaw_ + 1.57);

  //std::cout << p1 << " " << p2 << std::endl;
  Polygon polygon_view;
  //iterate within the FOV
  polygon_view.setFrameId(mapping_module_->map.getFrameId());
  polygon_view.addVertex(Position(p.position.x, p.position.y));
  polygon_view.addVertex(Position(p1.x, p1.y));
  polygon_view.addVertex(Position(p2.x, p2.y));
  polygon_view.addVertex(Position(p.position.x, p.position.y));

  double IG_view=0;
  for (grid_map::PolygonIterator iterator(mapping_module_->map, polygon_view);
       !iterator.isPastEnd(); ++iterator) {
        Index index=*iterator;
        IG_view+=getCellEntropy(index);
  }
  return IG_view;
}

void view_evaluator_IG::evaluate(){      //TOFIX project FOV from camera center not drone center

  info_selected_utility_ = 0; //- std::numeric_limits<float>::infinity(); //-inf
   selected_pose_.position.x = std::numeric_limits<double>::quiet_NaN();

   for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
    {
      Pose p = view_gen_->generated_poses[i];
       double utility = calculateIG(p);
       if (utility > info_selected_utility_)
       {
        info_selected_utility_ = utility;
         selected_pose_ = p;
       }
   }

       // No valid poses found, end
       if ( std::isnan(selected_pose_.position.x) )
       {
         return;
       }


}

Pose view_evaluator_IG::getTargetPose()
{
  return selected_pose_;
}




