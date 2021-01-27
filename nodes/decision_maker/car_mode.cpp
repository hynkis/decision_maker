#include <decision_maker_core.hpp>
namespace decision_maker


{
// ====================== //
// ----- IDLE group ----- //
// ====================== //

void DecisionMakerNode::entryInitState(const std::string& state_name, int status)
{
  ROS_INFO("----- Vehicle: State machine for vehicle system. -----");
}
void DecisionMakerNode::updateInitState(const std::string& state_name, int status)
{
  static bool is_first_callback = true;

  if (!is_first_callback)
  {
    return;
  }
  ROS_INFO("----- Vehicle: Eurecar is initializing now. -----");
  tryNextState("init_start");
  is_first_callback = false;
}

void DecisionMakerNode::entrySensorInitState(const std::string& state_name, int status)
{
  ROS_INFO("----- Vehicle: SensorInit start. -----");
  // Init something
//   Subs["lidar"] = nh_.subscribe("/merged/velodyne_points", 1, &DecisionMakerNode::callbackFromVelodyne, this);
}
void DecisionMakerNode::updateSensorInitState(const std::string& state_name, int status)
{
  // fake ready
  tryNextState("sensor_is_ready");

  if(isEventFlagTrue("Velodyne"))
  {
    publishOperatorHelpMessage(".");
    tryNextState("sensor_is_ready");
  }
  else{
    publishOperatorHelpMessage("Please run \"velodyne package\"");
  }  
}

void DecisionMakerNode::entryDriveReadyState(const std::string& state_name, int status)
{
  ROS_INFO("----- Vehicle: DriveReady start. -----");
  // Init something

//   ROS_INFO("----- Vehicle: Localization init start");
//   Subs["odometry"] = nh_.subscribe("/Odometry/ekf_estimated", 5, &DecisionMakerNode::callbackFromCurrentPose, this);
}
void DecisionMakerNode::updateDriveReadyState(const std::string& state_name, int status)
{
  // if human want to drive manually, switch to manual mode
  // ROS_INFO("Switch to the manual mode");
  // tryNextState("switch_manual_mode");

  // if goal is assigned, go to RoutePlanning
  ROS_INFO("Go to RoutePlanning");
  tryNextState("assign_goal");
}

void DecisionMakerNode::entryManualModeState(const std::string& state_name, int status)
{
  ROS_INFO("----- Vehicle: ManualMode start. -----");
  // Init something
}
void DecisionMakerNode::updateManualModeState(const std::string& state_name, int status)
{
  // Switch to Manual mode
  // --------

  // If human want autonomous mode, go back to DriveReady
  ROS_INFO("Go back to DriveReady");
  tryNextState("switch_auto_mode");
}

void DecisionMakerNode::entryRoutePlanningState(const std::string& state_name, int status)
{
  ROS_INFO("----- Vehicle: RoutePlanning start. -----");
  // Init something
}
void DecisionMakerNode::updateRoutePlanningState(const std::string& state_name, int status)
{
  // Request route planning
  // --------

  // If route is found, go to WaypointFollowing
  ROS_INFO("Route is found. Go to WaypointFollowing");
  tryNextState("found_route");
}

void DecisionMakerNode::entryWaypointFollowingState(const std::string& state_name, int status)
{
  ROS_INFO("----- Vehicle: WaypointFollowing start. -----");
  // Init something
}
void DecisionMakerNode::updateWaypointFollowingState(const std::string& state_name, int status)
{
  // Following local path with ACC
  // --------

  // If ego arrives goal, go back to DriveReady
  ROS_INFO("Arrives goal. Go back to DriveReady");
  tryNextState("arrive_goal");

  // If ego needs to overtake, go to Overtaking
  ROS_INFO("Need to overtake. Go to Overtaking");
  tryNextState("overtake");

  // If ego needs to avoid stopped obstacle, go to Avoidance
  ROS_INFO("Need to avoid. Go to Avoidance");
  tryNextState("avoid");

  // // If ego enters intersection, go to Intersection
  // ROS_INFO("Enter intersection. Go to Intersection");
  // tryNextState("enter_intersection");

  // // If ego (encounter pede or kick) and (not encounter traffic_cone)
  // ROS_INFO("Encounter Pede or Kick. Go to Pede_Kick");
  // tryNextState("encounter_pede_kick");

}

// ============================ //
// ----- OVERTAKING group ----- //
// ============================ //

void DecisionMakerNode::entryOvertakingState(const std::string& state_name, int status)
{
  ROS_INFO("----- Vehicle: Overtaking start. -----");
  // Init something
}
void DecisionMakerNode::updateOvertakingState(const std::string& state_name, int status)
{
  // Request larger Frenet Path planning
  // --------

  // If ego have done overtaking, go back to WaypointFollowing
  ROS_INFO("Done overtake. Go back to WaypointFollowing");
  tryNextState("overtake_done");

  // If ego does not have feasible Frenet path, go to Avoidance
  ROS_INFO("No feasible FrenetPath. Need to avoid from Overtaking state. Go to Avoidance");
  tryNextState("avoid");
}

// =========================== //
// ----- AVOIDANCE group ----- //
// =========================== //

void DecisionMakerNode::entryAvoidanceState(const std::string& state_name, int status)
{
  ROS_INFO("----- Vehicle: Avoidance start. -----");
  // Init something
}
void DecisionMakerNode::updateAvoidanceState(const std::string& state_name, int status)
{
  // Request Kynodynamic RRT* path
  // --------

  // If ego have done Avoidance, go back to WaypointFollowing
  ROS_INFO("Done avoid. Go back to WaypointFollowing");
  tryNextState("avoid_done");
}

}
