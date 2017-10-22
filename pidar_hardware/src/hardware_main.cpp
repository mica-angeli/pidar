#include <pidar_hardware/hardware_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

void controlLoop(pidar_hardware::PidarHW &pidar,
                  controller_manager::ControllerManager &cm,
                  time_source::time_point &last_time)
{
  // Calculate monotonic time difference
  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = this_time;

  // Process control loop
  pidar.reportLoopDuration(elapsed);
  pidar.updateJointsFromHardware();
  cm.update(ros::Time::now(), elapsed);
  pidar.writeCommandsToHardware();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pidar_hardware");
  ros::NodeHandle nh, private_nh("~");

  pidar_hardware::PidarHW pidar(nh, private_nh, 0.0);
  pidar.getInfo();
  return 0;
}
