#ifndef TEST_FLIGHT_H
#define TEST_FLIGHT_H

#include <victim_localization/common.h>

#include <victim_localization/victim_map_base.h>
#include <victim_localization/victim_map_dl.h>
#include <control/vehicle_control_base.h>
#include <control/vehicle_control_iris.h>

namespace NBVState {
enum State {
  INITIALIZING,
  IDLE,
  STARTING_ROBOT, STARTING_ROBOT_COMPLETE,
  UPDATE_MAP, UPDATE_MAP_COMPLETE,
  MOVING, MOVING_COMPLETE
};
}


class Test_Flight
{
public:
  VehicleControlBase *vehicle_;
  Victim_Map_Base *Map_;
  Test_Flight();
  NBVState::State state;
  bool is_done_map_update;
  int waypointNum;
  void runStateMachine();
  void initVehicle(void);
  void initMap(void);


};



#endif // TEST_FLIGHT_H




