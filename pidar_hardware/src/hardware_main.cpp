#include <pidar_hardware/hardware_interface.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pidar_hardware");
  ros::NodeHandle nh, private_nh("~");

  pidar_hardware::PidarHW pidar(nh, private_nh, 0.0);
  pidar.getInfo();
  return 0;
}
