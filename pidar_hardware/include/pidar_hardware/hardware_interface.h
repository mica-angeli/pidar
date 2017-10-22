#ifndef PIDAR_HARDWARE_HARDWARE_INTERFACE_H
#define PIDAR_HARDWARE_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <string.h>

#include <BrickPi3/BrickPi3.h>

namespace pidar_hardware {

  class PidarHW : public hardware_interface::RobotHW
  {
  public:
    PidarHW(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq);

    void updateJointsFromHardware();

    void writeCommandsToHardware();

    void getInfo();

  private:
    void initializeHardware();

    void registerControlInterfaces();

    BrickPi3 BP;

    ros::NodeHandle nh_, private_nh_;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    // ROS Parameters
    double wheel_diameter_, max_accel_, max_speed_;

    double polling_timeout_;

    uint8_t left_wheel_motor_port_;
    uint8_t right_wheel_motor_port_;

    struct Joint
    {
      double position;
      double position_offset;
      double velocity;
      double effort;
      double velocity_command;

      Joint() :
              position(0), velocity(0), effort(0), velocity_command(0)
      { }
    };

    Joint left_wheel_;
    Joint right_wheel_;

  };

} // namespace pidar_hardware
#endif //PIDAR_HARDWARE_HARDWARE_INTERFACE_H
