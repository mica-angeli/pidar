#include <pidar_hardware/hardware_interface.h>
#include <boost/assign/list_of.hpp>

namespace pidar_hardware {

  PidarHW::PidarHW(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq)
  :
  nh_(nh),
  private_nh_(private_nh),
  wheel_diameter_(0.0),
  max_accel_(0.0),
  max_speed_(0.0)
  {
    private_nh_.param("wheel_diameter", wheel_diameter_, wheel_diameter_);
    private_nh_.param("max_accel", max_accel_, max_accel_);
    private_nh_.param("max_speed", max_speed_, max_speed_);

    registerControlInterfaces();
  }

  void PidarHW::registerControlInterfaces()
  {
    // Initialize and register left wheel handles
    hardware_interface::JointStateHandle left_wheel_jsh("left_wheel",
                                                              &left_wheel_.position, &left_wheel_.velocity,
                                                              &left_wheel_.effort);
    joint_state_interface_.registerHandle(left_wheel_jsh);

    hardware_interface::JointHandle left_wheel_jh(left_wheel_jsh,
                                                  &left_wheel_.velocity_command);
    velocity_joint_interface_.registerHandle(left_wheel_jh);

    // Initialize and register right wheel handles
    hardware_interface::JointStateHandle right_wheel_jsh("right_wheel",
                                                        &right_wheel_.position, &right_wheel_.velocity,
                                                        &right_wheel_.effort);
    joint_state_interface_.registerHandle(right_wheel_jsh);

    hardware_interface::JointHandle right_wheel_jh(right_wheel_jsh,
                                                  &right_wheel_.velocity_command);
    velocity_joint_interface_.registerHandle(right_wheel_jh);

    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
  }


  void PidarHW::updateJointsFromHardware()
  {

  }

  void PidarHW::writeCommandsToHardware()
  {

  }

  void PidarHW::getInfo()
  {
    BP.detect();
    char str[33];
    BP.get_id(str);
    printf("Serial Number   : %s\n", str);
  }

}