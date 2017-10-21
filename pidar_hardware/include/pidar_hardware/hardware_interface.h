//
// Created by ricardo on 10/21/17.
//

#ifndef PIDAR_HARDWARE_HARDWARE_INTERFACE_H
#define PIDAR_HARDWARE_HARDWARE_INTERFACE_H

#include <BrickPi3/BrickPi3.h>

class PidarHW
{
public:
  void init();
private:
  BrickPi3 BP;

};


#endif //PIDAR_HARDWARE_HARDWARE_INTERFACE_H
