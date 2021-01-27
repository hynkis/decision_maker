# Eurecar Decision Maker

## Overview
**This feature is experimental.**</br>
Autoware package based state machine.
We adopted autoware package based state machine to make up of Eurecar decision maker node 

<img src="docs/Eurecar_state_vehicle.png" width=600>
<img src="docs/Eurecar_state_behavior.png" width=600>

## State Description

### Vehicle States
<img src="docs/Eurecar_state_vehicle.png" width=600>

### Vehicle States Description
State name|Required topic|Description|Implementation
--|--|---|--
Init|-|The parent state of the following states.|-
SensorInit|/merged/velodyne_points|Waits until all sensors are ready.|Waits until /merged/velodyne_points is received.  
LocalizationInit|/Odometry/ekf_estimated|Waits until localizer is ready | Waits until current_pose is converged.
PlanningInit|/closest_waypoint|Waits unil planners are ready | Subscriber is set for /closest_waypoint.
VehicleInit|-|Waits until vehicle is ready for departure.|No implementation goes directly to vehilce ready state.
VehicleReady|-|Vehicle is ready to move.|-

### Behavior States
<img src="docs/Eurecar_state_behavior.png" width=600>

### Behavior States Description
State name|Required topic|Description|Implementation
--|--|---|--
Init|-|The parent state of the following states.|-
GlobalPathInit|/based/lane_waypoints_raw|Waits until all path are ready.|Waits until waypoint is received.  
CostMapBasedPlanning|-|Base state in the motion planner| Graph based lattice planner using occupancy grid map.
Deceleration|/all_path_blocked|Decelerate before finding the collision path| Decelerate until vehicle speed is slower than safety speed.
HybridAstar|/all_path_blocked|Find a collision free path using hybrid A star|Find the collision free path and check costmap based planner is feasible. If it is feasible, relay costmap based planner


<!-- 
## ROS Parameters
Parameter|Type|Description
--|---|--
state_vehicle_file_name|string|file that defines vehicle state transition
state_mission_file_name|string|file that defines mission state transition
state_behavior_file_name|string|file that defines behavior state transition
state_motion_file_name|string|file that defines motion state transition
stopline_reset_count|int|This parameter is used if the vehicle stops at the stop line and moves backward without crossing the stop line. When the vehicle moves backward by this count of the waypoints, the stop line is recognized again. -->

<!-- 
## Subscribed topics
Topic|Type|Objective
--|---|--
/based/lane_waypoints_array|autoware_msgs/LaneArray|waypoints for the vehicle to follow. (e.g. waypoints given from waypoint_loader node)
/change_flag|std_msgs/Int32|Vehicle will try to change lane if this flag is set. Publishes /lamp_cmd to change blinkers according to this flag. (0=straight, 1=right, 2=left)
/closest_waypoint|std_msgs/Int32|Closest waypoint index in waypoints given by /based/lane_waypoints_array.
/config/decision_maker|autoware_config_msgs::ConfigDecisionMaker|Parameters set from runtime manager
/current_pose|geometry_msgs/PoseStamped|Current pose of vehicle
/current_velocity|geometry_msgs/TwistStamped|Current velocity of vehicle
/filtered_points|sensor_msgs/PointCloud2|Used to check if sensor data is ready. This is meant to give pointcloud data used for ndt_matching.
/final_waypoints|autoware_msgs/Lane|resultant waypoints from planner nodes, e.g. from velocity_set node.
/obstacle_waypoint|std_msgs/Int32|Obstacle waypoint index. Used in "Go" state.
/state_cmd|std_msgs/String|Decision_maker will try to transit state according to the key given through this topic.
/state/stop_order_wpidx|std_msgs/Int32|Vehicle will try to stop at this index of waypoint. Used in "OrderedStop" state.
/vector_map_info/area|vector_map_msgs/AreaArray|Area information from vector map. <br>This is ignored unless area, cross_road, line, point, road_sign, stop_line, and vector are subscribed.
/vector_map_info/cross_road|vector_map_msgs/CrossRoadArray|Cross road information from vector map. <br>This is ignored unless area, cross_road, line, point, road_sign, stop_line, and vector are subscribed.
/vector_map_info/line|vector_map_msgs/LineArray|Line information from vector map. <br>This is ignored unless area, cross_road, line, point, road_sign, stop_line, and vector are subscribed.
/vector_map_info/point|vector_map_msgs/PointArray|Point information from vector map.<br>This is ignored unless area, cross_road, line, point, road_sign, stop_line, and vector are subscribed.  
/vector_map_info/road_sign|vector_map_msgs/RoadSignArray|Road sign information from vector map. <br>This is ignored unless area, cross_road, line, point, road_sign, stop_line, and vector are subscribed.
/vector_map_info/stop_line|vector_map_msgs/StopLineArray|Stop line information from vector map.<br>This is ignored unless area, cross_road, line, point, road_sign, stop_line, and vector are subscribed.
/vector_map_info/vector|vector_map_msgs/VectorArray|Vector information from vector map. <br>This is ignored unless area, cross_road, line, point, road_sign, stop_line, and vector are subscribed.


## Published topics
Topic|Type|Objective
--|---|--
/decision_maker/available_transition|std_msgs/String|available transition from current state
/decision_maker/operator_help_text|jsk_rviz_plugins/OverlayText|Help message during operation
/decision_maker/state|std_msgs/String|current state for "Vehicle", "Mission", and "Drive" state machine.
/decision_maker/state_msg|autoware_msgs/State|current state for "Vehicle", "Mission", and "Drive" state machine with header.
/lamp_cmd|autoware_msgs/LampCmd|blinker command to vehicle (0=straight, 1=right, 2=left)
/lane_waypoints_array|autoware_msgs/LaneArray|waypoints passed down to following planners. (e.g. lane_rule)
/light_color_managed|autoware_msgs/TrafficLight|meant to publish light_color status. Not implemented yet.
/decision_maker/state_overlay|jsk_rviz_plugins/OverlayText|Current state as overlay_txt.
/state/stopline_wpidx|std_msgs/Int32|Index of waypoint for the vehicle to stop.
/decision_maker/target_velocity_array|std_msgs/Float64MultiArray| Array of target velocity obtained from final_waypoints.
/stop_location|autoware_msgs/VehicleLocation|Feedback to fms on the `/state_stop_order_wpidx` topic. It contains the index that the vehicle will stop and the id of the lane_array that the vehicle is using at the time. -->



