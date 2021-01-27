#ifndef __DECISION_MAKER_NODE__
#define __DECISION_MAKER_NODE__

#include <ros/ros.h>
#include <ros/spinner.h>

#include <unordered_map>
#include <stdio.h>
#include <random>
#include <string>
#include <cmath>
// #include <amathutils_lib/amathutils.hpp>

#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_rviz_plugins/OverlayText.h>

#include <state_machine_lib/state.hpp>
#include <state_machine_lib/state_context.hpp>

#include <ackermann_msgs/AckermannDriveStamped.h>


namespace decision_maker
{

class DecisionMakerNode
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Publishers
    std::unordered_map<std::string, ros::Publisher> Pubs;
    // Subscribers
    std::unordered_map<std::string, ros::Subscriber> Subs;
    // Event Flags to check callback is received.
    std::map<std::string, bool> EventFlags;
    //shared_ptr for received data from callback functions.  
    std::shared_ptr<ros::AsyncSpinner> spinners;

    std::shared_ptr<ackermann_msgs::AckermannDriveStamped> m_JoyCmdPtr;
    std::shared_ptr<ackermann_msgs::AckermannDriveStamped> m_WaypointTrackingCmdPtr;
    std::shared_ptr<ackermann_msgs::AckermannDriveStamped> m_VisualServoingCmdPtr;
    



  public:
    DecisionMakerNode(int argc, char** argv);
    ~DecisionMakerNode();

    void init(); //
    void run();
    void createSubscriber(); //
    void createPublisher(); //
    void tryNextState(const std::string& key); //

    void setEventFlag(const std::string& key, const bool& value);
    bool isEventFlagTrue(std::string key);
    void update(); //
    void setupStateCallback(); //

    // decision_maker_node_publish
    void update_publisher(void);
    void publishOperatorHelpMessage(const std::string& message);

    bool isSubscriberRegistered(const std::string& topic_name);

  public:
    state_machine::StateContext* ctx_car_mode;

  public:
    // // callback by topic subscribing
    void CallbackJoyControllerCmd(const ackermann_msgs::AckermannDriveStampedConstPtr& msg);
    void CallbackWaypointTrackingCmd(const ackermann_msgs::AckermannDriveStampedConstPtr& msg);
    void CallbackVisualServoingCmd(const ackermann_msgs::AckermannDriveStampedConstPtr& msg);

  private:
    /*** state of car_mode setting ***/
    // entry callback
    // ----- IDLE group
    void entryInitState(const std::string& state_name, int status);
    void entrySensorInitState(const std::string& state_name, int status);
    void entryDriveReadyState(const std::string& state_name, int status);
    void entryManualModeState(const std::string& state_name, int status);
    void entryRoutePlanningState(const std::string& state_name, int status);
    void entryWaypointFollowingState(const std::string& state_name, int status);
    // ----- OVERTAKING group
    void entryOvertakingState(const std::string& state_name, int status);
    // ----- AVOIDANCE group
    void entryAvoidanceState(const std::string& state_name, int status);
    
    // update callback
    // ----- IDLE group
    void updateInitState(const std::string& state_name, int status);
    void updateSensorInitState(const std::string& state_name, int status);
    void updateDriveReadyState(const std::string& state_name, int status);
    void updateManualModeState(const std::string& state_name, int status);
    void updateRoutePlanningState(const std::string& state_name, int status);
    void updateWaypointFollowingState(const std::string& state_name, int status);
    // ----- OVERTAKING group
    void updateOvertakingState(const std::string& state_name, int status);
    // ----- AVOIDANCE group
    void updateAvoidanceState(const std::string& state_name, int status);
    
    // exit callback



};

}  // namespace decision_maker

#endif
