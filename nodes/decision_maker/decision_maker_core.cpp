#include <decision_maker_core.hpp>

namespace decision_maker
{
DecisionMakerNode::DecisionMakerNode(int argc, char** argv) : private_nh_("~")
{
  std::string file_name_car_mode;
  private_nh_.getParam("car_mode", file_name_car_mode);
  
  ctx_car_mode = new state_machine::StateContext(file_name_car_mode, "car_mode");

  init();
}
DecisionMakerNode::~DecisionMakerNode(){}


void DecisionMakerNode::createSubscriber()
{
  // Config subscriber regarding control (including both lateral & longitudinal)
  Subs["JoyControllerCmd"] = nh_.subscribe("/Ackermann/command/joy", 1, &DecisionMakerNode::CallbackJoyControllerCmd, this);
  Subs["WaypointTrackingCmd"]  = nh_.subscribe("/Ackermann/command/wpt_tracking", 1, &DecisionMakerNode::CallbackWaypointTrackingCmd, this);
  Subs["VisualServoing"]  = nh_.subscribe("/Ackermann/command/visual_servoing", 1, &DecisionMakerNode::CallbackVisualServoingCmd, this);

}

void DecisionMakerNode::createPublisher()
{ 
  // for debug
  Pubs["operator_help_text"] = nh_.advertise<jsk_rviz_plugins::OverlayText>("operator_help_text", 1, true);
}

void DecisionMakerNode::init()
{
  createSubscriber();
  createPublisher();
  setupStateCallback();

  spinners = std::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(3));
  spinners->start();

  update_publisher();
}

void DecisionMakerNode::setEventFlag(const std::string& key, const bool& value)
{
  EventFlags[key] = value;
}

bool DecisionMakerNode::isEventFlagTrue(std::string key)
{
  if (EventFlags.count(key) == 0)
  {
    EventFlags[key] = false;
  }
  return EventFlags[key];
}

void DecisionMakerNode::tryNextState(const std::string& key)
{
  ctx_car_mode->nextState(key);
}

void DecisionMakerNode::update()
{
  update_publisher();
  if (ctx_car_mode)
  {
    ctx_car_mode->onUpdate();
  }
}

void DecisionMakerNode::CallbackJoyControllerCmd(const ackermann_msgs::AckermannDriveStampedConstPtr& msg)
{
    m_JoyCmdPtr = std::make_shared<ackermann_msgs::AckermannDriveStamped>(*msg);
    setEventFlag("JoyCmdReceived", true);
}

void DecisionMakerNode::CallbackWaypointTrackingCmd(const ackermann_msgs::AckermannDriveStampedConstPtr& msg)
{
    m_WaypointTrackingCmdPtr = std::make_shared<ackermann_msgs::AckermannDriveStamped>(*msg);
    setEventFlag("WptTrackingCmdReceived", true);
}

void DecisionMakerNode::CallbackVisualServoingCmd(const ackermann_msgs::AckermannDriveStampedConstPtr& msg)
{
    m_VisualServoingCmdPtr = std::make_shared<ackermann_msgs::AckermannDriveStamped>(*msg);
    setEventFlag("VisualServoingCmdReceived", true);
}


void DecisionMakerNode::run()
{
  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    update();

    loop_rate.sleep();
  }
}

bool DecisionMakerNode::isSubscriberRegistered(const std::string& topic_name)
{
  return Subs.count(topic_name) ? true : false;
}


void DecisionMakerNode::setupStateCallback()
{
  /*** state of car_mode setting ***/
  // ----- IDLE group
  ctx_car_mode->setCallback(state_machine::CallbackType::ENTRY, "Init",
                           std::bind(&DecisionMakerNode::entryInitState, this, std::placeholders::_1, 0));
  ctx_car_mode->setCallback(state_machine::CallbackType::UPDATE, "Init",
                           std::bind(&DecisionMakerNode::updateInitState, this, std::placeholders::_1, 0));

  ctx_car_mode->setCallback(state_machine::CallbackType::ENTRY, "SensorInit",
                           std::bind(&DecisionMakerNode::entrySensorInitState, this, std::placeholders::_1, 0));
  ctx_car_mode->setCallback(state_machine::CallbackType::UPDATE, "SensorInit",
                           std::bind(&DecisionMakerNode::updateSensorInitState, this, std::placeholders::_1, 0));

  ctx_car_mode->setCallback(state_machine::CallbackType::ENTRY, "DriveReady",
                           std::bind(&DecisionMakerNode::entryDriveReadyState, this, std::placeholders::_1, 0));
  ctx_car_mode->setCallback(state_machine::CallbackType::UPDATE, "DriveReady",
                           std::bind(&DecisionMakerNode::updateDriveReadyState, this, std::placeholders::_1, 0));

  ctx_car_mode->setCallback(state_machine::CallbackType::ENTRY, "ManualMode",
                           std::bind(&DecisionMakerNode::entryManualModeState, this, std::placeholders::_1, 0));
  ctx_car_mode->setCallback(state_machine::CallbackType::UPDATE, "ManualMode",
                           std::bind(&DecisionMakerNode::updateManualModeState, this, std::placeholders::_1, 0));

  ctx_car_mode->setCallback(state_machine::CallbackType::ENTRY, "RoutePlanning",
                           std::bind(&DecisionMakerNode::entryRoutePlanningState, this, std::placeholders::_1, 0));
  ctx_car_mode->setCallback(state_machine::CallbackType::UPDATE, "RoutePlanning",
                           std::bind(&DecisionMakerNode::updateRoutePlanningState, this, std::placeholders::_1, 0));

  ctx_car_mode->setCallback(state_machine::CallbackType::ENTRY, "WaypointFollowing",
                           std::bind(&DecisionMakerNode::entryWaypointFollowingState, this, std::placeholders::_1, 0));
  ctx_car_mode->setCallback(state_machine::CallbackType::UPDATE, "WaypointFollowing",
                           std::bind(&DecisionMakerNode::updateWaypointFollowingState, this, std::placeholders::_1, 0));
  
  // ----- OVERTAKING group
  ctx_car_mode->setCallback(state_machine::CallbackType::ENTRY, "Overtaking",
                           std::bind(&DecisionMakerNode::entryOvertakingState, this, std::placeholders::_1, 0));
  ctx_car_mode->setCallback(state_machine::CallbackType::UPDATE, "Overtaking",
                           std::bind(&DecisionMakerNode::updateOvertakingState, this, std::placeholders::_1, 0));
  
  // ----- AVOIDANCE group
  ctx_car_mode->setCallback(state_machine::CallbackType::ENTRY, "Avoidance",
                           std::bind(&DecisionMakerNode::entryAvoidanceState, this, std::placeholders::_1, 0));
  ctx_car_mode->setCallback(state_machine::CallbackType::UPDATE, "Avoidance",
                           std::bind(&DecisionMakerNode::updateAvoidanceState, this, std::placeholders::_1, 0));



  ctx_car_mode->nextState("started");

  // exit callback
}


jsk_rviz_plugins::OverlayText createOverlayText(const std::string& data, const int column)
{
  jsk_rviz_plugins::OverlayText ret;

  // message setup
  ret.width = 500;
  ret.height = 500;
  ret.top = 10 + (column * 500);
  ret.left = 10;
  ret.bg_color.r = 0;
  ret.bg_color.g = 0;
  ret.bg_color.b = 0;
  ret.bg_color.a = 0.8;

  ret.line_width = 2;
  ret.text_size = 9;
  ret.font = "DejaVu Sans Mono";
  ret.fg_color.r = 1.0;
  ret.fg_color.g = 1.0;
  ret.fg_color.b = 0.5;
  ret.fg_color.a = 0.9;

  ret.text = data;

  return ret;
}

void DecisionMakerNode::publishOperatorHelpMessage(const std::string& message)
{
  static std::vector<std::string> msg_log;
  static const size_t log_size = 4;

  msg_log.push_back(message);

  if (msg_log.size() >= log_size)
  {
    msg_log.erase(msg_log.begin());
  }

  std::string joined_msg;
  for (const auto& i : msg_log)
  {
    joined_msg += "> " + i + "\n";
  }
  Pubs["operator_help_text"].publish(createOverlayText(joined_msg, 0));
}

void DecisionMakerNode::update_publisher(void)
{
  // if (ctx_vehicle && ctx_behavior )
  // {
  //   // std::cout << "hello_update : " << ctx_vehicle->getStateText() << ", " << ctx_behavior->getStateText() << std::endl;
  //   static std::string text_vehicle_state, text_mission_state, text_behavior_state, text_motion_state;
  //   text_vehicle_state = ctx_vehicle->getStateText();
  //   text_behavior_state = ctx_behavior->getStateText();

  //   static std::string overlay_text;
  //   overlay_text = "> Vehicle:\n" + text_vehicle_state + 
  //                  "\n> Behavior:\n" + text_behavior_state;
  //   Pubs["state_overlay"].publish(createOverlayText(overlay_text, 1));

  //   static autoware_msgs::State state_array_msg;
  //   state_array_msg.header.stamp = ros::Time::now();
  //   state_array_msg.vehicle_state = text_vehicle_state;
  //   state_array_msg.behavior_state = text_behavior_state;
  //   Pubs["state_msg"].publish(state_array_msg);

  //   static std_msgs::String transition_msg;
  //   transition_msg.data = ctx_vehicle->getAvailableTransition(); 
  //   Pubs["available_transition"].publish(transition_msg);
  // }
  // else
  // {
  //   std::cerr << "ctx is not found " << std::endl;
  // }
}


} //namespace decision_maker
