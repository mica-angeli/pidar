#include <pidar_hardware/hardware_interface.h>

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