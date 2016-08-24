#ifndef POLLUX_H
#define POLLUX_H

// +--------------------------------------------------------+
// | A MICOS Pollox motor control library                   |
// | via RS232 interface.                                   |
// |                                                        |
// | Pollux CLASS HEADER                                    |
// |                                                        |
// | Benigno Gobbo                                          |
// | (c) INFN Sezione di Trieste                            |
// | V1.0.3                                                 |
// | 18 December 2007                                       |
// +--------------------------------------------------------+

#include <string>
#include <vector>
#include <fstream>
#include <termios.h>
#include <stdexcept>

class Pollux {
  
 public:

  // Constructor. Sets the serial port device and eventually 'time progress dots' too.
  Pollux( std::string port, bool writeDots = false );

  // Do nothing Destructor
  ~Pollux() {}

  // Connect and initialize serial port to comunicate with motor controller
  void  connectToPort( void ) throw( std::runtime_error );

  // Restore serial port to initial status
  void  restorePort( void ) throw( std::runtime_error );

  // Initialize the motor
  void  connectToDevice( int axis ) throw( std::runtime_error );

  // Reset the motor
  void  reset( int axis ) throw( std::runtime_error );

  // Clear the motor
  void  clear( int axis ) throw( std::runtime_error );

  // Move to origin and calibrate "0" position
  void  calibrate( int axis ) throw( std::runtime_error );

  // Move to end (about 51 mm) and measure the end position
  void  rangeMeasure( int axis ) throw( std::runtime_error );

  // Move "pos" mm away from current position 
  void  goToRelativePosition( int axis, double pos ) throw( std::runtime_error );
  void  goToRelativePosition2( int axis, double pos ) throw( std::runtime_error );

  // Move to "pos" mm absolute position (0<pos<50)
  void  goToAbsolutePosition( int axis, double pos ) throw( std::runtime_error );
  void  goToAbsolutePosition2( int axis, double pos ) throw( std::runtime_error );

  // Return the current positon in mm
  double getPosition( int axis ) throw( std::runtime_error );
  double getPosition2( int axis ) throw( std::runtime_error );

  // Return the mean of a motor controller error
  std::string getErrorString( int error );

  // Stop motion of an axis
  void stopMotion( int axis ) throw( std::runtime_error );
  
  // Serial Read
  std::string serialRead( void );

 private:

  std::string       _port;
  int               _fd;
  std::vector<bool> _connected;
  std::vector<int>  _devSN;
  struct termios    _oldtio;
  struct termios    _newtio;
  char              _buff[128];
  bool              _writeDots;

  void checkCompletion( int axis ) throw( std::runtime_error );

  int getError( int axis ) throw( std::runtime_error );

  int getNStack( void )  throw( std::runtime_error );

};

#endif // POLLUX_H
