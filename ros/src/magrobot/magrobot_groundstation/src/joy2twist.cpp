#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#include <map>
#include <string>

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
class TeleopTwistJoy
{
public:
  TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param);

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::Subscriber joy_sub;
  ros::Publisher cmd_vel_pub;

  int enable_button;
  int enable_turbo_button;

  std::map<std::string, int> axis_linear_map;
  std::map<std::string, double> scale_linear_map;
  std::map<std::string, double> scale_linear_turbo_map;

  std::map<std::string, int> axis_angular_map;
  std::map<std::string, double> scale_angular_map;
  std::map<std::string, double> scale_angular_turbo_map;

  bool sent_disable_msg;
};

/**
 * Constructs TeleopTwistJoy.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
TeleopTwistJoy::TeleopTwistJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{

  cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTwistJoy::joyCallback, this);

  nh_param->param<int>("enable_button", enable_button, 0);
  nh_param->param<int>("enable_turbo_button", enable_turbo_button, -1);

  if (nh_param->getParam("axis_linear", axis_linear_map))
  {
    nh_param->getParam("axis_linear", axis_linear_map);
    nh_param->getParam("scale_linear", scale_linear_map);
    nh_param->getParam("scale_linear_turbo", scale_linear_turbo_map);
  }
  else
  {
    nh_param->param<int>("axis_linear", axis_linear_map["x"], 1);
    nh_param->param<double>("scale_linear", scale_linear_map["x"], 0.5);
    nh_param->param<double>("scale_linear_turbo", scale_linear_turbo_map["x"], 1.0);
  }

  if (nh_param->getParam("axis_angular", axis_angular_map))
  {
    nh_param->getParam("axis_angular", axis_angular_map);
    nh_param->getParam("scale_angular", scale_angular_map);
    nh_param->getParam("scale_angular_turbo", scale_angular_turbo_map);
  }
  else
  {
    nh_param->param<int>("axis_angular", axis_angular_map["yaw"], 0);
    nh_param->param<double>("scale_angular", scale_angular_map["yaw"], 0.5);
    nh_param->param<double>("scale_angular_turbo",
    scale_angular_turbo_map["yaw"], scale_angular_map["yaw"]);
  }

  ROS_INFO_NAMED("TeleopTwistJoy", "Teleop enable button %i.", enable_button);
  ROS_INFO_COND_NAMED(enable_turbo_button >= 0, "TeleopTwistJoy",
      "Turbo on button %i.", enable_turbo_button);

  for (std::map<std::string, int>::iterator it = axis_linear_map.begin();
      it != axis_linear_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Linear axis %s on %i at scale %f.",
    it->first.c_str(), it->second, scale_linear_map[it->first]);
    ROS_INFO_COND_NAMED(enable_turbo_button >= 0, "TeleopTwistJoy",
        "Turbo for linear axis %s is scale %f.", it->first.c_str(), scale_linear_turbo_map[it->first]);
  }

  for (std::map<std::string, int>::iterator it = axis_angular_map.begin();
      it != axis_angular_map.end(); ++it)
  {
    ROS_INFO_NAMED("TeleopTwistJoy", "Angular axis %s on %i at scale %f.",
    it->first.c_str(), it->second, scale_angular_map[it->first]);
    ROS_INFO_COND_NAMED(enable_turbo_button >= 0, "TeleopTwistJoy",
        "Turbo for angular axis %s is scale %f.", it->first.c_str(), scale_angular_turbo_map[it->first]);
  }

  sent_disable_msg = false;
}

void TeleopTwistJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  // Initializes with zeros by default.
  geometry_msgs::Twist cmd_vel_msg;

  if (enable_turbo_button >= 0 && joy_msg->buttons[enable_turbo_button])
  {
    if (axis_linear_map.find("x") != axis_linear_map.end())
    {
      cmd_vel_msg.linear.x = joy_msg->axes[axis_linear_map["x"]] * scale_linear_turbo_map["x"];
    }
    if (axis_linear_map.find("y") != axis_linear_map.end())
    {
      cmd_vel_msg.linear.y = joy_msg->axes[axis_linear_map["y"]] * scale_linear_turbo_map["y"];
    }
    if  (axis_linear_map.find("z") != axis_linear_map.end())
    {
      cmd_vel_msg.linear.z = joy_msg->axes[axis_linear_map["z"]] * scale_linear_turbo_map["z"];
    }
    if  (axis_angular_map.find("yaw") != axis_angular_map.end())
    {
      cmd_vel_msg.angular.z = joy_msg->axes[axis_angular_map["yaw"]] * scale_angular_turbo_map["yaw"];
    }
    if  (axis_angular_map.find("pitch") != axis_angular_map.end())
    {
      cmd_vel_msg.angular.y = joy_msg->axes[axis_angular_map["pitch"]] * scale_angular_turbo_map["pitch"];
    }
    if  (axis_angular_map.find("roll") != axis_angular_map.end())
    {
      cmd_vel_msg.angular.x = joy_msg->axes[axis_angular_map["roll"]] * scale_angular_turbo_map["roll"];
    }

    cmd_vel_pub.publish(cmd_vel_msg);
    sent_disable_msg = false;
  }
  else if (joy_msg->buttons[enable_button])
  {
    if  (axis_linear_map.find("x") != axis_linear_map.end())
    {
      cmd_vel_msg.linear.x = joy_msg->axes[axis_linear_map["x"]] * scale_linear_map["x"];
    }
    if  (axis_linear_map.find("y") != axis_linear_map.end())
    {
      cmd_vel_msg.linear.y = joy_msg->axes[axis_linear_map["y"]] * scale_linear_map["y"];
    }
    if  (axis_linear_map.find("z") != axis_linear_map.end())
    {
      cmd_vel_msg.linear.z = joy_msg->axes[axis_linear_map["z"]] * scale_linear_map["z"];
    }
    if  (axis_angular_map.find("yaw") != axis_angular_map.end())
    {
      cmd_vel_msg.angular.z = joy_msg->axes[axis_angular_map["yaw"]] * scale_angular_map["yaw"];
    }
    if  (axis_angular_map.find("pitch") != axis_angular_map.end())
    {
      cmd_vel_msg.angular.y = joy_msg->axes[axis_angular_map["pitch"]] * scale_angular_map["pitch"];
    }
    if  (axis_angular_map.find("roll") != axis_angular_map.end())
    {
      cmd_vel_msg.angular.x = joy_msg->axes[axis_angular_map["roll"]] * scale_angular_map["roll"];
    }

    cmd_vel_pub.publish(cmd_vel_msg);
    sent_disable_msg = false;
  }
  else
  {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg)
    {
      cmd_vel_pub.publish(cmd_vel_msg);
      sent_disable_msg = true;
    }
  }
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "teleop_twist_joy_node");

  ros::NodeHandle nh(""), nh_param("~");
  TeleopTwistJoy joy_teleop(&nh, &nh_param);

  ros::spin();
}
