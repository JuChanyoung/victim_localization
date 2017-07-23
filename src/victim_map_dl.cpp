#include "victim_localization/victim_map_base.h"
#include "victim_localization/victim_map_dl.h"

victim_map_DL::victim_map_DL():
  Victim_Map_Base()
{
  setlayer_name("victim_DL");
  //default values for configs
   ros::param::param<double>("~In_DL_probability_of_detection_given_human", Prob_D_H , 0.9);
   ros::param::param<double>("~In_DL_probability_of_detection_given_no_human", Prob_D_Hc , 0.05);
   ros::param::param<double>("~In_DL_probability_of_no_detection_given_human", Prob_Dc_H , 0.1);
   ros::param::param<double>("~In_DL_probability_of_no_detection_given_no_human", Prob_Dc_Hc , 0.95);

 pub_map=nh_.advertise<grid_map_msgs::GridMap>(DL_map_topic, 1, true);
 pub_polygon=nh_.advertise<geometry_msgs::PolygonStamped>(DL_polygon_topic, 1, true);
}




