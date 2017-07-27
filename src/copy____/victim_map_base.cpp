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

 sub_loc = nh_.subscribe("/iris/mavros/local_position/pose", 100, &Victim_Map_Base::callbackdrawFOV, this);

}

Victim_Map_Base::~Victim_Map_Base(){}


grid_map::Polygon Victim_Map_Base::draw_FOV(){

  grid_map::Polygon polygon_FOV;

  //determine the triangle corner of the FOV
  p1.x= current_loc_.position.x+ max_depth_d*cos(current_yaw_) + const_* cos(current_yaw_ + 4.712);
  p2.x= current_loc_.position.x + max_depth_d*cos(current_yaw_) + const_* cos(current_yaw_ + 1.57);
  p1.y= current_loc_.position.y+ max_depth_d*sin(current_yaw_) + const_* sin(current_yaw_ + 4.712);
  p2.y= current_loc_.position.y + max_depth_d*sin(current_yaw_) + const_* sin(current_yaw_ + 1.57);

  //iterate within the FOV
  polygon_FOV.setFrameId(map.getFrameId());
  polygon_FOV.addVertex(Position(current_loc_.position.x, current_loc_.position.y));
  polygon_FOV.addVertex(Position(p1.x, p1.y));
  polygon_FOV.addVertex(Position(p2.x, p2.y));
  polygon_FOV.addVertex(Position(current_loc_.position.x, current_loc_.position.y));

  return polygon_FOV;
  //geometry_msgs::PolygonStamped message;
  //grid_map::PolygonRosConverter::toMessage(polygon, message);
  //pub_polygon.publish(message);
}

void Victim_Map_Base::callbackdrawFOV(const PoseStamped &ps_stamped){
  Pose ps=ps_stamped.pose;
  double yaw_=pose_conversion::getYawFromQuaternion(ps.orientation);
  Point p1_corner; // first corner point for triangle
  Point p2_corner; // second corner point for triangle
  //determine the triangle corner of the FOV
  p1_corner.x= ps.position.x+ max_depth_d*cos(yaw_) + const_* cos(yaw_ + 4.712);
  p2_corner.x= ps.position.x + max_depth_d*cos(yaw_) + const_* cos(yaw_ + 1.57);
  p1_corner.y= ps.position.y+ max_depth_d*sin(yaw_) + const_* sin(yaw_ + 4.712);
  p2_corner.y= ps.position.y + max_depth_d*sin(yaw_) + const_* sin(yaw_ + 1.57);

  grid_map::Polygon polygon_FOV;

  //iterate within the FOV
  polygon_FOV.setFrameId(map.getFrameId());
  polygon_FOV.addVertex(Position(ps.position.x, ps.position.y));
  polygon_FOV.addVertex(Position(p1_corner.x, p1_corner.y));
  polygon_FOV.addVertex(Position(p2_corner.x, p2_corner.y));
  polygon_FOV.addVertex(Position(ps.position.x, ps.position.y));


  //publish Field of View
  geometry_msgs::PolygonStamped message;
  grid_map::PolygonRosConverter::toMessage(polygon_FOV, message);
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

void Victim_Map_Base::setCurrentPose(Pose ps) {
  current_loc_=ps;
  current_yaw_=pose_conversion::getYawFromQuaternion(current_loc_.orientation);
}

void Victim_Map_Base::setDetectionResult(Point p, bool is_detect) {
  detect_loc_=p;
  is_detect_=is_detect;
}

 bool getDetectionStatus()
 {
   std::cout << "[Warning] Detection Checking not implemened in Based Map Module\n";
  }










