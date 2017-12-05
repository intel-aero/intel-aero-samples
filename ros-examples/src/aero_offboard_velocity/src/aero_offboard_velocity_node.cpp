/**
 * @file aero_offboard_velocity_node.cpp
 * @brief Demonstration of offboard velocity control in local NED and body
 * coordinates using MAVROS on Intel Aero, written with mavros version 0.14.2,
 * PX4 flight stack and tested in Gazebo SITL & jMAVSIM.
 */

#include <boost/thread/thread.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <time.h>
#include <unistd.h>

void saveCurrentStateCb(const mavros_msgs::StateConstPtr& msg, mavros_msgs::State* current_state)
{
  *current_state = *msg;
  bool connected = current_state->connected;
}

void waitForFCUConnection()
{
  mavros_msgs::State current_state;
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::Subscriber flight_mode_sub =
      node->subscribe<mavros_msgs::State>("mavros/state", 3, boost::bind(saveCurrentStateCb, _1, &current_state));
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
  }
}

class Offboard
{
private:
  ros::NodeHandle nh_;
  ros::Publisher set_vel_pub;
  Offboard() = delete;
  void setOffboardVelocityNED(float vx, float vy, float vz, float yaw);
  float convertToRadFromDeg(float deg);
  void setOffboardVelocityBody(float vx, float vy, float vz, float yaw_rate);

public:
  bool doOffbCtrlNED(void);
  Offboard(ros::NodeHandle* nodehandle);
  bool doOffbCtrlBody(void);
};

// Converting from degree to radian
float Offboard::convertToRadFromDeg(float deg)
{
  return (float)(deg / 180.0f * M_PI);
}

Offboard::Offboard(ros::NodeHandle* nodehandle)
{
  nh_ = *nodehandle;
  set_vel_pub = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
}

void Offboard::setOffboardVelocityBody(float vx, float vy, float vz, float yaw_rate)
{
  mavros_msgs::PositionTarget pos;
  pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

  pos.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                  mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                  mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                  mavros_msgs::PositionTarget::FORCE | mavros_msgs::PositionTarget::IGNORE_YAW;
  pos.position.x = 0.0f;
  pos.position.y = 0.0f;
  pos.position.z = 0.0f;
  pos.acceleration_or_force.x = 0.0f;
  pos.acceleration_or_force.y = 0.0f;
  pos.acceleration_or_force.z = 0.0f;
  pos.velocity.x = vx;
  pos.velocity.y = vy;
  pos.velocity.z = vz;
  pos.yaw = 0.0f;
  pos.yaw_rate = (float)convertToRadFromDeg(yaw_rate);
  set_vel_pub.publish(pos);
}

void Offboard::setOffboardVelocityNED(float vx, float vy, float vz, float yaw)
{
  mavros_msgs::PositionTarget pos;
  pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

  pos.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                  mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                  mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                  mavros_msgs::PositionTarget::FORCE | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  pos.position.x = 0.0f;
  pos.position.y = 0.0f;
  pos.position.z = 0.0f;
  pos.acceleration_or_force.x = 0.0f;
  pos.acceleration_or_force.y = 0.0f;
  pos.acceleration_or_force.z = 0.0f;
  pos.velocity.x = vx;
  pos.velocity.y = vy;
  pos.velocity.z = vz;
  pos.yaw = (float)convertToRadFromDeg(yaw);
  pos.yaw_rate = 0.0f;
  set_vel_pub.publish(pos);
}
bool Offboard::doOffbCtrlNED(void)
{
  // Send it once before starting offboard, otherwise it will be rejected.
  ROS_INFO("Zero velocity set");
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 0.0f);
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("Done with zero velocity set");

  ros::ServiceClient set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    ROS_INFO("OFFBOARD enabled");
  else
  {
    ROS_INFO("Unable to switch to offboard");
    return -1;
  }

  ROS_INFO("Turn to face East");
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 90.0f);
    ros::Duration(0.01).sleep();
  }

  ROS_INFO("Go North and back South");
  const float step_size = 0.01f;
  const float one_cycle = 2.0f * (float)M_PI;
  const unsigned steps = (unsigned)(one_cycle / step_size);
  for (unsigned i = 0; ros::ok() && i < steps; ++i)
  {
    int vx = 5.0f * sinf(i * step_size);
    setOffboardVelocityNED(0.0f, vx, 0.0f, 90.0f);
    ros::spinOnce();
    // rate.sleep();
    ros::Duration(0.01).sleep();
  }

  ROS_INFO("Turn to face West");
  for (int i = 400; ros::ok() && i > 0; --i)
  {
    setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 270.0f);
    ros::Duration(0.01).sleep();
  }

  ROS_INFO("Go up 2 m/s, turn to face South");
  for (int i = 400; ros::ok() && i > 0; --i)
  {
    setOffboardVelocityNED(0.0f, 0.0f, 2.0f, 180.0f);
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  ROS_INFO("Turn to face east");
  for (int i = 400; ros::ok() && i > 0; --i)
  {
    setOffboardVelocityNED(0.0f, 0.0f, 0.0f, 90.0f);
    ros::Duration(0.01).sleep();
  }

  ROS_INFO("Go down 1 m/s, turn to face North");
  for (int i = 400; ros::ok() && i > 0; --i)
  {
    setOffboardVelocityNED(0.0f, 0.0f, -1.0f, 0.0f);
    ros::Duration(0.01).sleep();
  }
  return true;
}

bool Offboard::doOffbCtrlBody(void)
{
  ROS_INFO("Zero velocity set");
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    setOffboardVelocityBody(0.0f, 0.0f, 0.0f, 0.0f);
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("Done with zero velocity set");

  ros::ServiceClient set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    ROS_INFO("OFFBOARD enabled");
  else
  {
    ROS_INFO("Unable to switch to offboard");
    return -1;
  }

  ROS_INFO("Turn around yaw clockwise and climb");
  for (int i = 200; ros::ok() && i > 0; --i)
  {
    setOffboardVelocityBody(0.0f, 0.0f, 1.0f, -60.0f);
    ros::Duration(0.01).sleep();
  }

  ROS_INFO("Turn yaw anti-clockwise");
  for (int i = 200; ros::ok() && i > 0; --i)
  {
    setOffboardVelocityBody(0.0f, 0.0f, 0.0f, 60.0f);
    ros::Duration(0.01).sleep();
  }

  ROS_INFO("Wait for a bit");
  for (int i = 200; ros::ok() && i > 0; --i)
  {
    setOffboardVelocityBody(0.0f, 0.0f, 0.0f, 0.0f);
    ros::Duration(0.01).sleep();
  }

  ROS_INFO("Fly a circle");
  for (int i = 500; ros::ok() && i > 0; --i)
  {
    setOffboardVelocityBody(5.0f, 5.0f, 0.0f, -60.0f);
    ros::Duration(0.01).sleep();
  }

  ROS_INFO("Wait for a bit");
  for (int i = 200; ros::ok() && i > 0; --i)
  {
    setOffboardVelocityBody(0.0f, 0.0f, 0.0f, 0.0f);
    ros::Duration(0.01).sleep();
  }

  ROS_INFO("Fly a circle sideways");
  for (int i = 500; ros::ok() && i > 0; --i)
  {
    setOffboardVelocityBody(0.0f, 5.0f, 0.0f, -60.0f);
    ros::Duration(0.01).sleep();
  }

  ROS_INFO("Wait for a bit");
  for (int i = 500; ros::ok() && i > 0; --i)
  {
    setOffboardVelocityBody(0.0f, 0.0f, 0.0f, 0.0f);
    ros::Duration(0.01).sleep();
  }
  return true;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_velocity_node");
  ros::NodeHandle nh;

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  Offboard offb(&nh);

  // Spawn the thread that handles FCU connection
  boost::thread thread_mode(waitForFCUConnection);

  // arming
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  mavros_msgs::CommandBool srv_arm;
  srv_arm.request.value = true;
  if (arming_client.call(srv_arm) && srv_arm.response.success)
  {
    ROS_INFO("ARM sent %d", srv_arm.response.success);
  }
  else
  {
    ROS_ERROR("Failed arming/disarming");
    return -1;
  }

  // takeoff
  ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = 490.0;
  srv_takeoff.request.latitude = 47.3977415;
  srv_takeoff.request.longitude = 8.5455937;
  srv_takeoff.request.min_pitch = 0;
  srv_takeoff.request.yaw = 0;
  if (takeoff_client.call(srv_takeoff) && srv_takeoff.response.success)
    ROS_INFO("Takeoff sent %d", srv_takeoff.response.success);
  else
  {
    ROS_ERROR("Failed Takeoff");
    return -1;
  }
  // let Aero reach given altitude
  sleep(5);

  // offboard NED velocity
  bool ret = offb.doOffbCtrlNED();
  if (ret == true)
    ROS_INFO("Done with NED velocity");
  else
    ROS_ERROR("Failed to run NED velocity");

  // land
  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  mavros_msgs::CommandTOL srv_land{};
  if (land_client.call(srv_land) && srv_land.response.success)
    ROS_INFO("Land sent %d", srv_land.response.success);
  else
  {
    ROS_ERROR("Landing failed");
    ros::shutdown();
    return -1;
  }
  sleep(5);

  // Takeoff
  if (takeoff_client.call(srv_takeoff) && srv_takeoff.response.success)
    ROS_INFO("Takeoff sent %d", srv_takeoff.response.success);
  else
  {
    ROS_ERROR("Failed Takeoff");
    return -1;
  }
  // let Aero reach given altitude
  sleep(5);

  // offboard body velocity
  ret = offb.doOffbCtrlBody();
  if (ret == true)
    ROS_INFO("Done with body  velocity");
  else
    ROS_ERROR("Failed to run body velocity");

  // land
  if (land_client.call(srv_land) && srv_land.response.success)
    ROS_INFO("Land sent %d", srv_land.response.success);
  else
  {
    ROS_ERROR("Landing failed");
    ros::shutdown();
    return -1;
  }

  return 0;
}
