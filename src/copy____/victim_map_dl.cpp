#include "victim_localization/victim_map_base.h"
#include "victim_localization/victim_map_dl.h"
#
victim_map_DL::victim_map_DL():
Victim_Map_Base()
{
  setlayer_name(DL_layer_name);
  // Create grid map
  map.setFrameId("map");
  map.setGeometry(Length(x_arena_max,y_arena_max), 1); //(Map is 20 by 20 meter with a resolution of 1m).
  ROS_INFO("Created Map with size %f x %f m (%i x %i cells).",
           map.getLength().x(), map.getLength().y(),
           map.getSize()(0), map.getSize()(1));


  map.add(layer_name,0.5);
  const_=max_depth_d/cos(DEG2RAD(HFOV_deg));

  pub_map=nh_.advertise<grid_map_msgs::GridMap>(DL_map_topic, 1, true);
  pub_polygon=nh_.advertise<geometry_msgs::PolygonStamped>(DL_polygon_topic, 1, true);

}

void victim_map_DL::Update(){

  polygon=draw_FOV();  //generate FOV_shape

  Position D_loc;
  D_loc[0]=detect_loc_.x;
  D_loc[1]=detect_loc_.y;
  D_loc=approximate_detect(D_loc);

  victim_found=false; //initialize detection to false

  for (grid_map::PolygonIterator iterator(map, polygon);
       !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    map.getPosition(index, position);

    float P_prior=map.at(layer_name, *iterator);

    if (position[0]== D_loc[0] && position[1]== D_loc[1])  {

      if (is_detect_== true && P_prior>0.01 ) {
        double Detec_prob=(Prob_D_H* P_prior)/((Prob_D_H* P_prior)+(Prob_D_Hc* (1-P_prior)));
        map.at(layer_name, index)= Detec_prob;
        if (Detec_prob>0) {
          victim_found=true;
          victim_loc=position;
        }
      }

      if (is_detect_== true && P_prior<0.01 ) map.at(layer_name, index)=(Prob_D_H* 0.5)/((Prob_D_H* 0.5)+(Prob_D_Hc*0.5));

      if (is_detect_== false )  map.at(layer_name, index)=(Prob_Dc_H* P_prior)/((Prob_Dc_H* P_prior)+(Prob_Dc_Hc* (1-P_prior)));
    }
    else  {map.at(layer_name, index)=(Prob_Dc_H* P_prior)/((Prob_Dc_H* P_prior)+(Prob_Dc_Hc* (1-P_prior))); }

  }
  publish_Map();
}

bool getDetectionStatus(){
  return victim_found;
}








