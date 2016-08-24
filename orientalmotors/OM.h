#ifndef OM_H
#define OM_H

// +--------------------------------------------------------+
// | A Oriental Motor EZS II motor control library          |
// | via RS232 interface.                                   |
// |                                                        |
// | Benigno Gobbo                                          |
// | (c) INFN Sezione di Trieste                            |
// | V1.0                                                   |
// | 15 October 2013                                        |
// +--------------------------------------------------------+

#include <string>
#include <vector>
#include <fstream>
#include <termios.h>
#include <stdexcept>

class OM {
  
 public:

  // Constructor
  OM( std::string device );

  // Do nothing Destructor
  ~OM() {}

  // connect to device 
  void connectToDevice( void ) throw( std::runtime_error );

  // restore serial port to initial state
  void restoreDevice( void ) throw( std::runtime_error );

  // Set devide ID
  //void setID( std::string id ) throw( std::runtime_error );

  // Initialization
  //void init( std::string id ) throw( std::runtime_error );
  void init(  int limb = 600 ) throw( std::runtime_error );

  // Select a device
  //void selectDevice( std::string id ) throw( std::runtime_error );

  // Send command
  std::string sendCommandToMotor( std::string command ) throw( std::runtime_error );

  // Get motor position
  double getMotorPosition( void ) throw( std::runtime_error );

  // Get firmware version
  std::string getVersion( void ) throw( std::runtime_error );

  // Absolute motion
  double moveAbs( double abs, bool noWait = false )  throw( std::runtime_error );

  // Incremental motion
  double moveInc( double inc, bool noWait = false )  throw( std::runtime_error );

  // Reset
  std::string reset( void ) throw( std::runtime_error );

  //void waitForUnlock( void );
  //inline void unlock( void ) { _motorsCommunicationLocked = false; }

 private:

  //static bool       _motorsCommunicationLocked;
  std::string       _device;
  int               _fd;
  //std::string       _id;
  char              _buff[128];
  struct termios    _oldtio;
  struct termios    _newtio;

  // Motor go home
  void goHome( bool noWait = false ) throw( std::runtime_error );

  // Get response
  std::string getResponseFromMotor( void ) throw( std::runtime_error );

  // Serial read
  std::string serialRead( void );

};

#endif // OM_H
