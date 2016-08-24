// +--------------------------------------------------------+
// | To move motors                                         |
// | Benigno Gobbo                                          |
// | (c) INFN Sezione di Trieste                            |
// | V1.0                                                   |
// | 16 July 2015                                           |
// +--------------------------------------------------------+

#ifndef Motors_H
#define Motors_H

#include <vector>
#include <string>
#include <sstream>

class Pollux;
class OM;

class Motors {

 public:

  Motors( void );
  ~Motors( void ) {}

  double updateMotorPosition( int device );
  void initializeMotors( void );
  double moveMotorAbs( int device, double position );
  double moveMotorInc( int device, double increment );
  void stopMotors( void );
  void resetMotor( int device );
  
  inline bool motorsFound( void ) { return( _motorDevicesFound ); }
  inline bool motorsInitialized( void ) { return( _initialized ); }

  
 private:

  bool _motorDevicesFound;
  bool _initialized;
  std::vector<std::string> _omDevices;
  std::vector<std::string> _pxDevices;
  OM*     _om[2];
  Pollux* _px;

  bool searchMotorDevices( void );
  
};

#endif // Motors_H
