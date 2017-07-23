#include <victim_localization/victim_map_base.h>

using namespace grid_map;


Victim_Map_Base::Victim_Map_Base()
{
  //default values for configs
  ros::param::param<double>("~probability_of_detection_given_human", Prob_D_H , 0.9);
  ros::param::param<double>("~probability_of_detection_given_no_human", Prob_D_Hc , 0.05);
  ros::param::param<double>("~probability_of_no_detection_given_human", Prob_Dc_H , 0.1);
  ros::param::param<double>("~probability_of_no_detection_given_no_human", Prob_Dc_Hc , 0.95);
  ros::param::param<double>("~HFOV_angle", HFOV_deg , 29);
  ros::param::param<double>("~maximum_depth_distance", max_depth_d , 4);
  ros::param::param<double>("~maximum_arena_width", x_arena_max , 20);
  ros::param::param<double>("~maximum_arena_height", y_arena_max , 20);
  setlayer_name("victim");


  // Create grid map
  map.setFrameId("map");
  map.setGeometry(Length(x_arena_max,y_arena_max), 1); //(Map is 20 by 20 meter with a resolution of 1m).
  ROS_INFO("Created %s with size %f x %f m (%i x %i cells).",
           layer_name, map.getLength().x(), map.getLength().y(),
           map.getSize()(0), map.getSize()(1));

  map.add(layer_name,0.5);

  const_=max_depth_d/cos(DEG2RAD(HFOV_deg));

}

Victim_Map_Base::~Victim_Map_Base(){}


//new stuff//
void Victim_Map_Base::Update(Pose c,Pose d,bool is_detect){

  current_loc_=c;
  detect_loc_=d;
  is_detect_=is_detect;

  draw_FOV();  //generate FOV_shape

  Position D_loc;
  D_loc[0]=detect_loc_.position.x;
  D_loc[1]=detect_loc_.position.y;
  D_loc=approximate_detect(D_loc);

  for (grid_map::PolygonIterator iterator(map, polygon);
       !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    map.getPosition(index, position);

    float P_prior=map.at(layer_name, *iterator);

    if (position[0]== D_loc[0] && position[1]== D_loc[1])  {

      if (is_detect_== true && P_prior>0.01 ) map.at(layer_name, index)=(Prob_D_H* P_prior)/((Prob_D_H* P_prior)+(Prob_D_Hc* (1-P_prior)));
      if (is_detect_== true && P_prior<0.01 ) map.at(layer_name, index)=(Prob_D_H* 0.5)/((Prob_D_H* 0.5)+(Prob_D_Hc*0.5));

      if (is_detect_== false )  map.at(layer_name, index)=(Prob_Dc_H* P_prior)/((Prob_Dc_H* P_prior)+(Prob_Dc_Hc* (1-P_prior)));
    }
    else  {map.at(layer_name, index)=(Prob_Dc_H* P_prior)/((Prob_Dc_H* P_prior)+(Prob_Dc_Hc* (1-P_prior))); }

  }
  publish_Map();
}

void Victim_Map_Base::draw_FOV(){

  //determine the triangle corner of the FOV
  p1.x= current_loc_.position.x+ max_depth_d*cos(0) + const_* cos(0 + 4.712);
  p2.x= current_loc_.position.x + max_depth_d*cos(0) + const_* cos(0 + 1.57);
  p1.y= current_loc_.position.y+ max_depth_d*sin(0) + const_* sin(0 + 4.712);
  p2.y= current_loc_.position.y + max_depth_d*sin(0) + const_* sin(0 + 1.57);

  //iterate within the FOV
  polygon.setFrameId(map.getFrameId());
  polygon.addVertex(Position(current_loc_.position.x, current_loc_.position.y));
  polygon.addVertex(Position(p1.x, p1.y));
  polygon.addVertex(Position(p2.x, p2.y));
  polygon.addVertex(Position(current_loc_.position.x, current_loc_.position.y));

  geometry_msgs::PolygonStamped message;
  grid_map::PolygonRosConverter::toMessage(polygon, message);
  pub_polygon.publish(message);
}




void Victim_Map_Base::publish_Map(){
  ros::Time time = ros::Time::now();
  map.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(map, message);
  pub_map.publish(message);
  ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
}


/*  bool valid (Position loc) {
  bool valid;
  Index index;
  map.getIndex(loc,index);
  float boundary_x=x_arena_max/2-0.1;
  float boundary_y=y_arena_max/2-0.1;

  if (((loc[0] >= -boundary_x) && (loc[0] <= boundary_x)) && ((loc[1] >= -boundary_y) && (loc[1]  <= boundary_y))) valid=true; // this is to check for NAN values
   else valid=false;
   return valid;
}
*/
Position Victim_Map_Base::approximate_detect(Position res){
  Position approx_;
  if (int(res[0])==0) if (res[0]>0) approx_[0]=0.5; else approx_[0]=-0.5;
  if (int(res[0])>0) approx_[0]= int(res[0]) + 0.5;
  if (int(res[0])<0) approx_[0]= int(res[0]) - 0.5;

  if (approx_[0] > 10) approx_[0]=9.9;  if (approx_[0] < -10) approx_[0]=-9.9;

  if (int(res[1])==0) if (res[1]>0) approx_[1]=0.5; else approx_[1]=-0.5;
  if (int(res[1])>0) approx_[1]= int(res[1]) + 0.5;
  if (int(res[1])<0) approx_[1]= int(res[1]) - 0.5;

  if (approx_[1] > 10) approx_[1]=9.9;  if (approx_[1] < -10) approx_[1]=-9.9;

  return approx_;
}

std::string Victim_Map_Base::getlayer_name() {
  return layer_name;
}
void Victim_Map_Base::setlayer_name(std::string layer_) {
  layer_name=layer_;
}



