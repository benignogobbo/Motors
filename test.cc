#include <iostream>
#include <string>
#include <stdexcept>
#include <sstream>
#include <unistd.h>

#include "Motors.h"

// +----------+
// |   main   |
// +----------+

int main( int argc, char **argv ) {

  Motors motors;
  
  bool status = motors.motorsFound();
  std::cout << "Motor Devices Found Status: " << status << std::endl;

  motors.initializeMotors();

  status = motors.motorsInitialized();
  std::cout << "Motor Devices Initialization Status: " << status << std::endl;
  
  std::cout << "Move om X axis to 500" << std::endl;
  double pos = motors.moveMotorAbs( 0, 500 );
  std::cout << "OM X axis final position: " << pos << std::endl;

  std::cout << "Move om Y axis to 500" << std::endl;
  pos = motors.moveMotorAbs( 1, 500 );
  std::cout << "OM Y axis final position: " << pos << std::endl;

  std::cout << "Move px X axis to 2.5" << std::endl;
  pos = motors.moveMotorAbs( 2, 2.5 );
  std::cout << "px X axis final position: " << pos << std::endl;

  std::cout << "Move px Y axis to 2.5" << std::endl;
  pos = motors.moveMotorAbs( 3, 2.5 );
  std::cout << "px Y axis final position: " << pos << std::endl;

  
  return 0;

}

