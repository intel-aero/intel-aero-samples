/**
 * @file aero_offboard_velocity_node.cpp
 * @brief MAVROS Offboard Control example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL & jMAVSIM.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <mavros_msgs/CommandTOL.h>
#include <time.h>
#include <boost/thread/thread.hpp>
#include <ros/duration.h>

void state_cb(const mavros_msgs::StateConstPtr& msg, mavros_msgs::State* current_state)
{
  *current_state = *msg;
  bool connected = current_state->connected;
}

void wait_For_FCU_Connection()
{
  mavros_msgs::State current_state;
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::Subscriber flight_mode_sub =
      node->subscribe<mavros_msgs::State>("mavros/state", 3, boost::bind(state_cb, _1, &current_state));
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
  }
}

class Offboard
{
public:
  Offboard();
  Offboard(ros::NodeHandle* nodehandle);
  ros::NodeHandle nh_;
  void set_Offboard_Velocity_Ned(float vx, float vy, float vz, float yaw);
  void arm_drone();
  ros::Publisher set_vel_pub;
  bool offb_Ctrl_Ned(void);
  float to_Rad_From_Deg(float deg);
  void set_Offboard_Velocity_Body(float vx, float vy, float vz, float yaw_rate);
  bool offb_Ctrl_Body(void);
};

// Converting from degree to radian
float Offboard::to_Rad_From_Deg(float deg)
{
  return (float)(deg / 180.0f * M_PI);
}

Offboard::Offboard(ros::NodeHandle* nodehandle)
{
  nh_ = *nodehandle;
  set_vel_pub = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
}

void Offboard::set_Offboard_Velocity_Body(float vx, float vy, float vz, float yaw_rate)
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
  pos.yaw_rate = (float)to_Rad_From_Deg(yaw_rate);
  set_vel_pub.publish(pos);
}

void Offboard::set_Offboard_Velocity_Ned(float vx, float vy, float vz, float yaw)
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
  pos.yaw = (float)to_Rad_From_Deg(yaw);
  pos.yaw_rate = 0.0f;
  set_vel_pub.publish(pos);
}
bool Offboard::offb_Ctrl_Ned(void)
{
  // Send it once before starting offboard, otherwise it will be rejected.
  ROS_INFO("zero velocity set");
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    set_Offboard_Velocity_Ned(0.0f, 0.0f, 0.0f, 0.0f);
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
    ROS_INFO("unable to switch to offboard");
    return -1;
  }

  ROS_INFO("Turn to face East");
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    set_Offboard_Velocity_Ned(0.0f, 0.0f, 0.0f, 90.0f);
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("done");

  ROS_INFO(" Go North and back South");
  const float step_size = 0.01f;
  const float one_cycle = 2.0f * (float)M_PI;
  const unsigned steps = (unsigned)(one_cycle / step_size);
  for (unsigned i = 0; ros::ok() && i < steps; ++i)
  {
    int vx = 5.0f * sinf(i * step_size);
    set_Offboard_Velocity_Ned(0.0f, vx, 0.0f, 90.0f);
    ros::spinOnce();
    // rate.sleep();
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("done");

  ROS_INFO("Turn to face West");
  for (int i = 400; ros::ok() && i > 0; --i)
  {
    set_Offboard_Velocity_Ned(0.0f, 0.0f, 0.0f, 270.0f);
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("Done ");

  ROS_INFO("Go up 2 m/s, turn to face South");
  for (int i = 400; ros::ok() && i > 0; --i)
  {
    set_Offboard_Velocity_Ned(0.0f, 0.0f, 2.0f, 180.0f);
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("Done ");

  ROS_INFO("turn to face east");
  for (int i = 400; ros::ok() && i > 0; --i)
  {
    set_Offboard_Velocity_Ned(0.0f, 0.0f, 0.0f, 90.0f);
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("Done ");

  ROS_INFO("Go down 1 m/s, turn to face North");
  for (int i = 400; ros::ok() && i > 0; --i)
  {
    set_Offboard_Velocity_Ned(0.0f, 0.0f, -1.0f, 0.0f);
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("Done");
  return true;
}

bool Offboard::offb_Ctrl_Body(void)
{
  ROS_INFO("zero velocity set");
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    set_Offboard_Velocity_Body(0.0f, 0.0f, 0.0f, 0.0f);
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
    ROS_INFO("unable to switch to offboard");
    return -1;
  }

  ROS_INFO("Turn around yaw clockwise and climb");
  for (int i = 200; ros::ok() && i > 0; --i)
  {
    set_Offboard_Velocity_Body(0.0f, 0.0f, 1.0f, -60.0f);
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("Done");

  ROS_INFO("turn yaw anti-clockwise");
  for (int i = 200; ros::ok() && i > 0; --i)
  {
    set_Offboard_Velocity_Body(0.0f, 0.0f, 0.0f, 60.0f);
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("Done");

  ROS_INFO("wait for a bit");
  for (int i = 200; ros::ok() && i > 0; --i)
  {
    set_Offboard_Velocity_Body(0.0f, 0.0f, 0.0f, 0.0f);
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("Done");

  ROS_INFO("fly a circle");
  for (int i = 500; ros::ok() && i > 0; --i)
  {
    set_Offboard_Velocity_Body(5.0f, 5.0f, 0.0f, -60.0f);
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("Done");

  ROS_INFO("wait for a bit");
  for (int i = 200; ros::ok() && i > 0; --i)
  {
    set_Offboard_Velocity_Body(0.0f, 0.0f, 0.0f, 0.0f);
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("Done");

  ROS_INFO("fly a circle sideways");
  for (int i = 500; ros::ok() && i > 0; --i)
  {
    set_Offboard_Velocity_Body(0.0f, 5.0f, 0.0f, -60.0f);
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("Done");

  ROS_INFO("wait for a bit");
  for (int i = 500; ros::ok() && i > 0; --i)
  {
    set_Offboard_Velocity_Body(0.0f, 0.0f, 0.0f, 0.0f);
    ros::Duration(0.01).sleep();
  }
  ROS_INFO("Done");
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_velocity_node");
  ros::NodeHandle nh;

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  Offboard offb_obj(&nh);

  boost::thread thread_mode(wait_For_FCU_Connection);

  // arming
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  mavros_msgs::CommandBool srv_arm;
  srv_arm.request.value = true;
  if (arming_client.call(srv_arm) && srv_arm.response.success)
    ROS_INFO("ARM sent %d", srv_arm.response.success);
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
    ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
  else
  {
    ROS_ERROR("Failed Takeoff");
    return -1;
  }

  sleep(5);

  // offboard ned velocity
  bool ret = offb_obj.offb_Ctrl_Ned();
  if (ret == true)
  {
    printf("Done with Ned velocity");
  }

  // land
  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  mavros_msgs::CommandTOL srv_land{};
  if (land_client.call(srv_land) && srv_land.response.success)
    ROS_INFO("land sent %d", srv_land.response.success);
  else
  {
    ROS_ERROR("Landing failed");
    ros::shutdown();
    return -1;
  }
  sleep(5);

  // Takeoff
  if (takeoff_client.call(srv_takeoff) && srv_takeoff.response.success)
    ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
  else
  {
    ROS_ERROR("Failed Takeoff");
    return -1;
  }

  sleep(5);

  // offboard body velocity
  ret = offb_obj.offb_Ctrl_Body();
  if (ret == true)
  {
    printf("done with body  velocity");
  }

  // land
  if (land_client.call(srv_land) && srv_land.response.success)
    ROS_INFO("land sent %d", srv_land.response.success);
  else
  {
    ROS_ERROR("Landing failed");
    ros::shutdown();
    return -1;
  }

  return 0;
}
