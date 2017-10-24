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
  pidar.updateJointsFromHardware(elapsed);
  cm.update(ros::Time::now(), elapsed);
  pidar.writeCommandsToHardware();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pidar_hardware");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency;
  private_nh.param("control_frequency", control_frequency, 10.0);

  pidar_hardware::PidarHW pidar(nh, private_nh, control_frequency);
  controller_manager::ControllerManager cm(&pidar, nh);
  pidar.getInfo();

  ros::CallbackQueue pidar_queue;
  ros::AsyncSpinner pidar_spinner(1, &pidar_queue);

  time_source::time_point last_time = time_source::now();
  ros::TimerOptions control_timer(
          ros::Duration(1 / control_frequency),
          boost::bind(controlLoop, boost::ref(pidar), boost::ref(cm), boost::ref(last_time)),
          &pidar_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  pidar_spinner.start();

  ros::spin();

  return 0;
}
