#include <victim_localization/test_flight.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>


Test_Flight::Test_Flight()
{
    // >>>>>>>>>>>>>>>>>
    // Initialize ROS
    // >>>>>>>>>>>>>>>>>
    state = NBVState::INITIALIZING;
    // initialize some parameters
    is_done_map_update=false;
    waypointNum=0;
    ROS_INFO("Test: Begin!");
}


void Test_Flight::initVehicle(){

     vehicle_ = new VehicleControlIris();

    // Set starting position
    ros::Duration(2.0).sleep();
    std::cout << "vehilce..." << std::endl;
    int pose_number;
    std::string pose_number_str;
    ros::param::param<int>("~uav_start_pose", pose_number, 1);
    pose_number_str = std::to_string(pose_number);

    double x, y, z, yaw;
    ros::param::param<double>("~uav_pose_x_" + pose_number_str, x, -8);
    ros::param::param<double>("~uav_pose_y_" + pose_number_str, y, -8);
    ros::param::param<double>("~uav_pose_z_" + pose_number_str, z, 1.5);
    ros::param::param<double>("~uav_pose_yaw_" + pose_number_str, yaw, 0);

    vehicle_->setWaypoint(x, y, z, yaw);

    printf("Starting vehicle\n");
    vehicle_->start();
    printf("Moving vehicle\n");
    vehicle_->moveVehicle(0.25);
    printf("Done moving\n");
  }

void Test_Flight::initMap(){
  int map_type;
    ros::param::param("~map_type", map_type, 0);

    switch(map_type)
    {
    default:
    case 0:
      Map_ = new victim_map_DL();
      break;
    }

}

void Test_Flight::runStateMachine(){
  ROS_INFO("test_loop: Starting vehicle. Waiting for current position information.");
 state = NBVState::STARTING_ROBOT;

 ros::Rate loop_rate(30);
   while (ros::ok())
   {
     switch(state)
     {
       case NBVState::IDLE:
       break;

       case NBVState::STARTING_ROBOT:
         if( vehicle_->isReady() )
         {
           state = NBVState::STARTING_ROBOT_COMPLETE;
           break;
         }
         vehicle_->start();
         break;

       case (NBVState::STARTING_ROBOT_COMPLETE):
       case(NBVState::UPDATE_MAP_COMPLETE):

          if(waypointNum==0) vehicle_->setWaypoint(-8.0,-4.0,1.5,0.0);
          if(waypointNum==1) vehicle_->setWaypoint(-8.0,0.0,1.5,0.0);
          if(waypointNum==2) vehicle_->setWaypoint(-8.0,4.0,1.5,0.0);
          if(waypointNum==3) vehicle_->setWaypoint(-8.0,8.0,1.5,0.0);
          if(waypointNum>=4) {state = NBVState::IDLE; break;}

         waypointNum++;
        state = NBVState::MOVING;
         break;


       case NBVState::MOVING:
         vehicle_->moveVehicle(0.25);
         state = NBVState::UPDATE_MAP;
         break;

       case NBVState::UPDATE_MAP:
         // Update history
         Map_->Update(vehicle_->getPose(),vehicle_->getPose(),false);
         state = NBVState::UPDATE_MAP_COMPLETE;
         break;
     }

     ros::spinOnce();
     loop_rate.sleep();
 }
}


int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "test_");

  //Create an object of class  Victim_mapping that will take care of everything
  Test_Flight *test_;
  std::cout << "Starting...\n";
  test_->initVehicle();
  std::cout << "pass InitializationVehicle\n";
  test_->initMap();
  std::cout << "pass InitializationMap\n";


  test_->runStateMachine();

  return 0;
}
