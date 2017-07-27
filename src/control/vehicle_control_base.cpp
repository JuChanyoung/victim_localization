#include <ros/ros.h>
#include "control/vehicle_control_base.h"
//#include "victim_localization/common.h"

VehicleControlBase::VehicleControlBase():
  is_ready_(false)
{
  ros::param::param("~distance_threshold", distance_threshold_, 0.4);
  ros::param::param("~angular_threshold", angular_threshold_, DEG2RAD(10.0));
  ros::param::param("~linear_speed_threshold", linear_speed_threshold_, 0.05);
  ros::param::param("~angular_speed_threshold", angular_speed_threshold_, 0.1);  //before 0.03
}


geometry_msgs::Pose VehicleControlBase::getPose()
{
  return vehicle_current_pose_;
}

geometry_msgs::Twist VehicleControlBase::getTwist()
{
  return vehicle_current_twist_;
}

geometry_msgs::Point VehicleControlBase::getPosition()
{
  return vehicle_current_pose_.position;
}

mavros_msgs::State VehicleControlBase::getState()
{
  return vehicle_current_state_;
}


geometry_msgs::Quaternion VehicleControlBase::getOrientation()
{
  vehicle_current_pose_.orientation;
}

double VehicleControlBase::getYaw()
{
  return pose_conversion::getYawFromQuaternion( getOrientation() );
}


double VehicleControlBase::getDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
  return sqrt(
    (p1.position.x-p2.position.x)*(p1.position.x-p2.position.x) +
    (p1.position.y-p2.position.y)*(p1.position.y-p2.position.y) +
    (p1.position.z-p2.position.z)*(p1.position.z-p2.position.z) );
}

double VehicleControlBase::getAngularDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
  double yaw1 = pose_conversion::getYawFromQuaternion(p1.orientation);
  double yaw2 = pose_conversion::getYawFromQuaternion(p2.orientation);

  // Set differnce from -pi to pi
  double yaw_diff = fmod(yaw1 - yaw2, 2*M_PI);

  if (yaw_diff > M_PI)
    yaw_diff = yaw_diff - 2*M_PI;

  else if (yaw_diff < -M_PI)
    yaw_diff = yaw_diff + 2*M_PI;

  //std::cout << yaw1 << " " << yaw2 <<std::endl;
  //std::cout << "yaw_diff: " << yaw_diff <<std::endl;
  return yaw_diff;
}

bool VehicleControlBase::isNear(double p1, double p2, double threshold_sensitivity )
{
  if (fabs(p1-p2)< distance_threshold_*threshold_sensitivity)
  {
    return true;
  }
  return false;
}

bool VehicleControlBase::isNear(const geometry_msgs::Pose p_target, const geometry_msgs::Pose p_current, double threshold_sensitivity){

  // std::cout << fabs(getAngularDistance(p_target, p_current)) << "< " <<  angular_threshold_*threshold_sensitivity << std::endl;
  // std::cout << getDistance(p_target, p_current) << "< " <<  distance_threshold_*threshold_sensitivity << std::endl;
  // std::cout << "theshold Dis: " << threshold_sensitivity << std::endl;

  if (
    getDistance(p_target, p_current) < distance_threshold_*threshold_sensitivity &&
    fabs(getAngularDistance(p_target, p_current)) < angular_threshold_*threshold_sensitivity )
  {
    return true;
  }

  return false;
}
