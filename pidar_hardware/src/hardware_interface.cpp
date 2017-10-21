//
// Created by ricardo on 10/21/17.
//

#include <pidar_hardware/hardware_interface.h>

void PidarHW::init()
{
  BP.detect();
  char str[33];
  BP.get_id(str);
  printf("Serial Number   : %s\n", str);
}

int main(void)
{
  PidarHW pidar;
  pidar.init();
  printf("HelloBrickWorld\n");
  return 0;
}