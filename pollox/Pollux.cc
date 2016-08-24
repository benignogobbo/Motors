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

#include <mutex>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>
#include <sys/time.h>
#include <sstream>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "Pollux.h"

std::mutex & polluxMut() {
  static std::mutex m;
  return m;
}

std::mutex & polluxMutSR() {
  static std::mutex m;
  return m;
}

// +-------------+
// | constructor |
// +-------------+

Pollux::Pollux( std::string port, bool writeDots ) {
  _port       = port;
  _writeDots  = writeDots;
  _fd         = 0;
  _connected.push_back( false ); // 1st axis
  _connected.push_back( false ); // 2nd axis
  _devSN.push_back( 4030163 );
  _devSN.push_back( 4030164 );
}


// +------------------------------------------------------+
// | connect to device with correct settings              |
// +------------------------------------------------------+

void Pollux::connectToPort( void ) throw( std::runtime_error ) {
  
  //_fd = open( _port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
  _fd = open( _port.c_str(), O_RDWR | O_NOCTTY );
  if( _fd<0 ) { 
    std::stringstream s;
    s << strerror( errno ); 
    throw( std::runtime_error( s.str() ) );
    return; 
  }
    
  tcgetattr( _fd, &_oldtio );
 
  _newtio = _oldtio;
  
  _newtio.c_cflag  = (B19200);

  tcflush( _fd, TCIOFLUSH );
  tcsetattr( _fd, TCSANOW, &_newtio );  

  // Wait a second, otherwise it does not work...
  sleep(1);

  return;
}


// +------------------------------------------------------+
// | restore port settings to old values                  |
// +------------------------------------------------------+

void Pollux::restorePort( void ) throw( std::runtime_error ) {

  tcsetattr( _fd, TCSANOW, &_oldtio ); /* restore old port settings */
  if( close( _fd ) < 0 ) {
    std::stringstream s;
    s << strerror( errno ); 
    throw ( std::runtime_error( s.str() ) );
  } 

  return;
}


// +------------------------------------------------------+
// | initialization and calibration                       |
// +------------------------------------------------------+

void Pollux::connectToDevice( int axis ) throw( std::runtime_error ) {

  if( _fd == 0 ) {
    throw( std::runtime_error( "Not yet connected to communication port" ) );
    return;
  }  

  if( axis < 1 || axis > 2 ) {
    throw( std::runtime_error( "Axis must be '1' or '2'" ) );
    return;
  }

  tcflush( _fd, TCIOFLUSH );
  
  // Check if the device is there...

  std::stringstream getserialno;
  getserialno << axis << " getserialno ";

  int status = write( _fd, getserialno.str().c_str(), getserialno.str().size() );
  if( status != getserialno.str().size() ) {
    throw( std::runtime_error( "Error in sending 'get serial number' command" ) );
    return;
  }

  usleep(100000);
  
  std::string sr = serialRead();
  int serialno = 0;
  std::stringstream s;
  s << sr;
  s >> serialno;

  // Check if serial number is known...
  std::vector<int>::iterator f = find( _devSN.begin(), _devSN.end(), serialno );
  if( f == _devSN.end() ) {
    throw( std::runtime_error( "Error in checking device serial number" ) );
    return;
  }

  usleep(100000);

  _connected[axis-1] = true;

  return;

}

// +------------------------------------------------------+
// | reset                                                |
// +------------------------------------------------------+

void Pollux::reset( int axis ) throw( std::runtime_error ) {

  if( axis < 1 || axis > 2 ) {
    throw( std::runtime_error( "Axis must be '1' or '2'" ) );
    return;
  }

  tcflush( _fd, TCIOFLUSH );

  std::stringstream nreset;
  nreset << axis << " nreset ";

  int status = write( _fd, nreset.str().c_str(), nreset.str().size() );
  if( status != nreset.str().size() ) {
    throw( std::runtime_error( "Error in sending 'reset' command" ) );
    return;
  }

  usleep(100000); 

  return;

}

// +------------------------------------------------------+
// | clear                                                |
// +------------------------------------------------------+

void Pollux::clear( int axis ) throw( std::runtime_error ) {

  if( axis < 1 || axis > 2 ) {
    throw( std::runtime_error( "Axis must be '1' or '2'" ) );
    return;
  }

  tcflush( _fd, TCIOFLUSH );

  std::stringstream nclear;
  nclear << axis << " nclear ";

  int status = write( _fd, nclear.str().c_str(), nclear.str().size() );
  if( status != nclear.str().size() ) {
    throw( std::runtime_error( "Error in sending 'clear' command" ) );
    return;
  }

  usleep(100000);

  return;

}

// +------------------------------------------------------+
// | calibrate the motor                                  |
// +------------------------------------------------------+

void Pollux::calibrate( int axis ) throw( std::runtime_error ) {

  std::lock_guard<std::mutex> _( polluxMut() );
  
  if( axis < 1 || axis > 2 ) {
    throw( std::runtime_error( "Axis must be '1' or '2'" ) );
    return;
  }

  tcflush( _fd, TCIOFLUSH );

  std::stringstream command;
  command << axis << " ncalibrate ";

  int status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'calibrate' command" ) );
    return;
  }

  //usleep(100000);

  //try{
  //  checkCompletion( axis );
  //} catch( std::runtime_error &e ) {
  //  throw( std::runtime_error( e.what() ) );
  //  return;
  //}

  command.clear();
  command.str( std::string() );
  command << axis << " nstatus ";
  
  std::string sr;
  do {
    status = write( _fd, command.str().c_str(), command.str().size() );
    if( status != command.str().size() ) {
      throw( std::runtime_error( "Error in sending 'get status' command" ) );
      return;
    }
    usleep(200000);
    sr = serialRead();
    if( sr.substr(0,1) != "0" && sr.substr(0,1) != "1" ) {
      std::stringstream s;
      s << "Error in getting device status: " << _buff[0];
      throw( std::runtime_error( s.str() ) );
      return;
    } 
    usleep(100000);
  } while( sr.substr(0,1) != "0" );

  usleep(100000);

  int error = 0;

  //try{
  //  error = getError( axis );
  //} catch( std::runtime_error &e ) {
  //  throw( std::runtime_error( e.what() ) );
  //  return;
  //}

  command.clear();
  command.str( std::string() );
  command << axis << " getnerror ";

  status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'get error' command" ) );
    return;
  }
  
  usleep(100000);
  
  sr = serialRead();
  
  std::stringstream s;
  s << sr;
  s >> error;

  if( error > 0 && error != 1004 ) {
    std::stringstream s;
    s << "Device internal error. " << this->getErrorString( error );
    throw( std::runtime_error( s.str() ) );
    return;
  }

  return;

}

// +------------------------------------------------------+
// | measure the range                                    |
// +------------------------------------------------------+

void Pollux::rangeMeasure( int axis ) throw( std::runtime_error ) {

  std::lock_guard<std::mutex> _( polluxMut() );

  if( axis < 1 || axis > 2 ) {
    throw( std::runtime_error( "Axis must be '1' or '2'" ) );
    return;
  }

  tcflush( _fd, TCIOFLUSH );

  std::stringstream command;
  command << axis << " nrangemeasure ";

  int status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'range measure' command" ) );
    return;
  }
  
  usleep(100000);

  //try{
  //  checkCompletion( axis );
  //} catch( std::runtime_error &e ) {
  //  throw( std::runtime_error( e.what() ) );
  //  return;
  //}

  command.clear();
  command.str( std::string() );
  command << axis << " nstatus ";
  
  std::string sr;
  do {
    status = write( _fd, command.str().c_str(), command.str().size() );
    if( status != command.str().size() ) {
      throw( std::runtime_error( "Error in sending 'get status' command" ) );
      return;
    }
    usleep(200000);
    sr = serialRead();
    if( sr.substr(0,1) != "0" && sr.substr(0,1) != "1" ) {
      std::stringstream s;
      s << "Error in getting device status: " << _buff[0];
      throw( std::runtime_error( s.str() ) );
      return;
    } 
    usleep(100000);
  } while( sr.substr(0,1) != "0" );

  usleep(100000);

  int error = 0;

  //try{
  //  error = getError( axis );
  //} catch( std::runtime_error &e ) {
  //  throw( std::runtime_error( e.what() ) );
  //  return;
  //}

  command.clear();
  command.str( std::string() );
  command << axis << " getnerror ";

  status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'get error' command" ) );
    return;
  }
  
  usleep(100000);
  
  sr = serialRead();
  
  std::stringstream s;
  s << sr;
  s >> error;

  if( error > 0 && error != 1004 && error != 1002 ) {
    std::stringstream s;
    s << "Device internal error. " << this->getErrorString( error );
    throw( std::runtime_error( s.str() ) );
    return;
  }

  return;

}


// +------------------------------------------------------+
// | move "pos" mm from relative position                 |
// +------------------------------------------------------+

void Pollux::goToRelativePosition( int axis, double pos ) throw( std::runtime_error ) {

  std::lock_guard<std::mutex> _( polluxMut() );

  if( axis < 1 || axis > 2 ) {
    throw( std::runtime_error( "Axis must be '1' or '2'" ) );
    return;
  }

  if( ! _connected[axis-1] ) {
    try {
      this->connectToDevice( axis );
    } catch( std::runtime_error &e ) {
      std::cout << e.what() << std::endl;
      return;
    }
  }

  tcflush( _fd, TCIOFLUSH );

  std::stringstream command;
  command << std::setiosflags( std::ios::showpoint ) << pos << " " << axis << " nrmove ";

  int status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'move to relative position' command" ) );
    return;
  }
  
  usleep(100000);
  
  //try{
  //  checkCompletion( axis );
  //} catch( std::runtime_error &e ) {
  //  throw( std::runtime_error( e.what() ) );
  //  return;
  //}

  command.clear();
  command.str( std::string() );
  command << axis << " nstatus ";
  
  std::string sr;
  do {
    status = write( _fd, command.str().c_str(), command.str().size() );
    if( status != command.str().size() ) {
      throw( std::runtime_error( "Error in sending 'get status' command" ) );
      return;
    }
    usleep(200000);
    sr = serialRead();
    if( sr.substr(0,1) != "0" && sr.substr(0,1) != "1" ) {
      std::stringstream s;
      s << "Error in getting device status: " << _buff[0];
      throw( std::runtime_error( s.str() ) );
      return;
    } 
    usleep(100000);
  } while( sr.substr(0,1) != "0" );
  
  usleep(100000);

  int error = 0;

  //try{
  //  error = getError( axis );
  //} catch( std::runtime_error &e ) {
  //  throw( std::runtime_error( e.what() ) );
  //  return;
  //}

  command.clear();
  command.str( std::string() );
  command << axis << " getnerror ";

  status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'get error' command" ) );
    return;
  }
  
  usleep(100000);
  
  sr = serialRead();
  
  std::stringstream s;
  s << sr;
  s >> error;

  if( error > 0 ) {
    std::stringstream s;
    s << "Device internal error. " << this->getErrorString( error );
    if( error != 1009 ) throw( std::runtime_error( s.str() ) );
    return;
  }

  return; 
 
}

// +------------------------------------------------------+
// | move "pos" mm from relative position, version 2      |
// +------------------------------------------------------+

void Pollux::goToRelativePosition2( int axis, double pos ) throw( std::runtime_error ) {

  std::lock_guard<std::mutex> _( polluxMut() );

  if( axis < 1 || axis > 2 ) {
    throw( std::runtime_error( "Axis must be '1' or '2'" ) );
    return;
  }

  if( ! _connected[axis-1] ) {
    try {
      this->connectToDevice( axis );
    } catch( std::runtime_error &e ) {
      std::cout << e.what() << std::endl;
      return;
    }
  }

  tcflush( _fd, TCIOFLUSH );

  // clear parameters stack
  std::stringstream command;
  command << axis << " nclear ";
  int status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'clear' command" ) );
    return;
  }

  // get current position
  double oldPosition;
  command.clear();
  command.str( std::string() );
  command << axis << " npos ";
  status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'get position' command" ) );
    return;
  }
  std::string sr = serialRead();
  std::stringstream s; s << sr;
  s >> oldPosition;

  // move
  command.clear();
  command.str( std::string() );
  command << std::setiosflags( std::ios::showpoint ) << pos << " " << axis << " nrmove ";
  status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'move to relative position' command" ) );
    return;
  }

  // wait the end of the movement
  command.clear();
  command.str( std::string() );
  command << axis << " nstatus ";
  do {
    status = write( _fd, command.str().c_str(), command.str().size() );
    if( status != command.str().size() ) {
      throw( std::runtime_error( "Error in sending 'get status' command" ) );
      return;
    }
    usleep(200000);
    sr = serialRead();
    if( sr.substr(0,1) != "0" && sr.substr(0,1) != "1" ) {
      std::stringstream s;
      s << "Error in getting device status: " << _buff[0];
      throw( std::runtime_error( s.str() ) );
      return;
    } 
  } while( sr.substr(0,1) != "0" );
  
  // check for errors
  int error = 0;
  command.clear();
  command.str( std::string() );
  command << axis << " getnerror ";
  status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'get error' command" ) );
    return;
  }
  sr = serialRead();
  s.clear();
  s.str( std::string() );
  s << sr;
  s >> error;
  if( error > 0 ) {
    std::stringstream s;
    s << "Device internal error. " << this->getErrorString( error );
    if( error != 1009 ) throw( std::runtime_error( s.str() ) );
    return;
  }

  // get new position
  double newPosition;
  command.clear();
  command.str( std::string() );
  command << axis << " npos ";
  status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'get position' command" ) );
    return;
  }
  sr = serialRead();
  s.clear();
  s.str( std::string() );
  s << sr;
  s >> newPosition;

  double positioningError = fabs( ( newPosition - oldPosition ) - pos );
  if( positioningError > 1.0e-8 ) {
    throw( std::runtime_error( "Error in positioning bigger than 10 nm" ) );
    return;
  }

  return; 
 
}

// +------------------------------------------------------+
// | move to absolute position "pos"                      |
// +------------------------------------------------------+

void Pollux::goToAbsolutePosition( int axis, double pos ) throw( std::runtime_error ) {

  std::lock_guard<std::mutex> _( polluxMut() );

  if( axis < 1 || axis > 2 ) {
    throw( std::runtime_error( "Axis must be '1' or '2'" ) );
    return;
  }

  if( ! _connected[axis-1] ) {
    try {
      this->connectToDevice( axis );
    } catch( std::runtime_error &e ) {
      std::cout << e.what() << std::endl;
      return;
    }
  }

  if( pos < 0.0 || pos > 50.0 ) {
    throw( std::runtime_error( "Position must be between 0.0 and 50.0 mm" ) );
    return;
  }

  tcflush( _fd, TCIOFLUSH );

  std::stringstream command;
  command << std::setiosflags( std::ios::showpoint ) << pos << " " << axis << " nmove ";

  int status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'move to absolute position' command" ) );
    return;
  }
  
  usleep(100000);
  
  //try{
  //  checkCompletion( axis );
  //} catch( std::runtime_error &e ) {
  //  throw( std::runtime_error( e.what() ) );
  //  return;
  //}

  command.clear();
  command.str( std::string() );
  command << axis << " nstatus ";
  
  std::string sr;
  do {
    status = write( _fd, command.str().c_str(), command.str().size() );
    if( status != command.str().size() ) {
      throw( std::runtime_error( "Error in sending 'get status' command" ) );
      return;
    }
    usleep(200000);
    sr = serialRead();
    if( sr.substr(0,1) != "0" && sr.substr(0,1) != "1" ) {
      std::stringstream s;
      s << "Error in getting device status: " << _buff[0];
      throw( std::runtime_error( s.str() ) );
      return;
    } 
    usleep(100000);
  } while( sr.substr(0,1) != "0" );
  
  usleep(100000);

  int error = 0;

  //try{
  //  error = getError( axis );
  //} catch( std::runtime_error &e ) {
  //  throw( std::runtime_error( e.what() ) );
  //  return;
  //}

  command.clear();
  command.str( std::string() );
  command << axis << " getnerror ";

  status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'get error' command" ) );
    return;
  }
  
  usleep(100000);
  
  sr = serialRead();
  
  std::stringstream s;
  s << sr;
  s >> error;

  if( error > 0 && error != 1004 ) { 
    std::stringstream s;
    s << "Device internal error. " << this->getErrorString( error );
    throw( std::runtime_error( s.str() ) );
    return;
  }
    
  return;
  
}

// +------------------------------------------------------+
// | move to absolute position "pos", version 2           |
// +------------------------------------------------------+

void Pollux::goToAbsolutePosition2( int axis, double pos ) throw( std::runtime_error ) {

  std::lock_guard<std::mutex> _( polluxMut() );

  if( axis < 1 || axis > 2 ) {
    throw( std::runtime_error( "Axis must be '1' or '2'" ) );
    return;
  }

  if( ! _connected[axis-1] ) {
    try {
      this->connectToDevice( axis );
    } catch( std::runtime_error &e ) {
      std::cout << e.what() << std::endl;
      return;
    }
  }

  if( pos < 0.0 || pos > 50.0 ) {
    throw( std::runtime_error( "Position must be between 0.0 and 50.0 mm" ) );
    return;
  }

  tcflush( _fd, TCIOFLUSH );

  // clear parameters stack
  std::stringstream command;
  command << axis << " nclear ";
  int status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'clear' command" ) );
    return;
  }

  // move
  command.clear();
  command.str( std::string() );
  command << std::setiosflags( std::ios::showpoint ) << pos << " " << axis << " nmove ";
  status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'move to relative position' command" ) );
    return;
  }

  // wait the end of the movement
  command.clear();
  command.str( std::string() );
  command << axis << " nstatus ";  
  std::string sr;
  do {
    status = write( _fd, command.str().c_str(), command.str().size() );
    if( status != command.str().size() ) {
      throw( std::runtime_error( "Error in sending 'get status' command" ) );
      return;
    }
    usleep(200000);
    sr = serialRead();
    if( sr.substr(0,1) != "0" && sr.substr(0,1) != "1" ) {
      std::stringstream s;
      s << "Error in getting device status: " << _buff[0];
      throw( std::runtime_error( s.str() ) );
      return;
    } 
    usleep(100000);
  } while( sr.substr(0,1) != "0" );

  // check for errors  
  int error = 0;
  command.clear();
  command.str( std::string() );
  command << axis << " getnerror ";
  status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'get error' command" ) );
    return;
  }
  sr = serialRead();
  std::stringstream s;
  s << sr;
  s >> error;
  if( error > 0 && error != 1004 ) { 
    std::stringstream s;
    s << "Device internal error. " << this->getErrorString( error );
    throw( std::runtime_error( s.str() ) );
    return;
  }
    
  // get new position

  double newPosition;
  command.clear();
  command.str( std::string() );
  command << axis << " npos ";
  status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'get position' command" ) );
    return;
  }
  sr = serialRead();
  s.clear();
  s.str( std::string() );
  s << sr;
  s >> newPosition;

  double positioningError = fabs( newPosition - pos );
  if( positioningError > 1.0e-8 ) {
    throw( std::runtime_error( "Error in positioning bigger than 10 nm" ) );
    return;
  }

  return;
  
}

// +------------------------------------------------------+
// | stop motion of a given axis                          |
// +------------------------------------------------------+

void Pollux::stopMotion( int axis ) throw( std::runtime_error ) {

  if( axis < 1 || axis > 2 ) {
    throw( std::runtime_error( "Axis must be '1' or '2'" ) );
    return;
  }

  if( ! _connected[axis-1] ) {
    try {
      this->connectToDevice( axis );
    } catch( std::runtime_error &e ) {
      std::cout << e.what() << std::endl;
      return;
    }
  }

  tcflush( _fd, TCIOFLUSH );

  std::stringstream command;
  //command << axis << " stopspeed ";
  command << axis << " nabort ";

  int status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'terminate constant velocity move' command" ) );
    return;
  }
  
  usleep(100000);
  
  //try{
  //  checkCompletion( axis );
  //} catch( std::runtime_error &e ) {
  //  throw( std::runtime_error( e.what() ) );
  //  return;
  //}

  command.clear();
  command.str( std::string() );
  command << axis << " nstatus ";
  
  std::string sr;
  do {
    status = write( _fd, command.str().c_str(), command.str().size() );
    if( status != command.str().size() ) {
      throw( std::runtime_error( "Error in sending 'get status' command" ) );
      return;
    }
    usleep(200000);
    sr = serialRead();
    if( sr.substr(0,1) != "0" && sr.substr(0,1) != "1" ) {
      std::stringstream s;
      s << "Error in getting device status: " << _buff[0];
      throw( std::runtime_error( s.str() ) );
      return;
    } 
    usleep(100000);
  } while( sr.substr(0,1) != "0" );
  
  usleep(100000);

  int error = 0;

  //try{
  //  error = getError( axis );
  //} catch( std::runtime_error &e ) {
  //  throw( std::runtime_error( e.what() ) );
  //  return;
  //}

  command.clear();
  command.str( std::string() );
  command << axis << " getnerror ";

  status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'get error' command" ) );
    return;
  }
  
  usleep(100000);
  
  sr = serialRead();
  
  std::stringstream s;
  s << sr;
  s >> error;

  if( error > 0 && error != 1004 ) { 
    std::stringstream s;
    s << "Device internal error. " << this->getErrorString( error );
    throw( std::runtime_error( s.str() ) );
    return;
  }

  return;
  
}

// +------------------------------------------------------+
// | get motor position                                   |
// +------------------------------------------------------+

double Pollux::getPosition( int axis ) throw( std::runtime_error ) {

  std::lock_guard<std::mutex> _( polluxMut() );

  double position = 0.0;
  
  if( axis < 1 || axis > 2 ) {
    throw( std::runtime_error( "Axis must be '1' or '2'" ) );
    return position;
  }

  if( ! _connected[axis-1] ) {
    try {
      this->connectToDevice( axis );
    } catch( std::runtime_error &e ) {
      std::cout << e.what() << std::endl;
      return position;
    }
  }
  
  tcflush( _fd, TCIOFLUSH );

  std::stringstream npos;
  npos << axis << " npos ";

  int status = write( _fd, npos.str().c_str(), npos.str().size() );
  if( status != npos.str().size() ) {
    throw( std::runtime_error( "Error in sending 'get position' command" ) );
    return position;
  }

  usleep(100000);
  
  std::string sr = serialRead();
  std::stringstream s;
  s << sr;
  s >> position;

  usleep(100000);

  return position;
  
}

// +------------------------------------------------------+
// | get motor position, version 2                        |
// +------------------------------------------------------+

double Pollux::getPosition2( int axis ) throw( std::runtime_error ) {

  std::lock_guard<std::mutex> _( polluxMut() );

  double position = 0.0;
  
  if( axis < 1 || axis > 2 ) {
    throw( std::runtime_error( "Axis must be '1' or '2'" ) );
    return position;
  }

  if( ! _connected[axis-1] ) {
    try {
      this->connectToDevice( axis );
    } catch( std::runtime_error &e ) {
      std::cout << e.what() << std::endl;
      return position;
    }
  }
  
  tcflush( _fd, TCIOFLUSH );

  // clear parameters stack
  std::stringstream command;
  command << axis << " nclear ";
  int status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'clear' command" ) );
    return position;
  }

  // move
  command.clear();
  command.str( std::string() );
  command << axis << " npos ";

  status = write( _fd, command.str().c_str(), command.str().size() );
  if( status != command.str().size() ) {
    throw( std::runtime_error( "Error in sending 'get position' command" ) );
    return position;
  }
  std::string sr = serialRead();
  std::stringstream s;
  s << sr;
  s >> position;

  return position;
  
}
 
// +------------------------------------------------------+
// | get error code                                       |
// +------------------------------------------------------+

int Pollux::getError( int axis ) throw( std::runtime_error ) {

  std::lock_guard<std::mutex> _( polluxMut() );

  int error = 0;

  if( axis < 1 || axis > 2 ) {
    throw( std::runtime_error( "Axis must be '1' or '2'" ) );
    return 9999;
  }
  
  if( ! _connected[axis-1] ) {
    try {
      this->connectToDevice( axis );
    } catch( std::runtime_error &e ) {
      std::cout << e.what() << std::endl;
      return 9999;
    }
  }

  tcflush( _fd, TCIOFLUSH );
  
  std::stringstream getnerror;
  getnerror << axis << " getnerror ";

  int status = write( _fd, getnerror.str().c_str(), getnerror.str().size() );
  if( status != getnerror.str().size() ) {
    throw( std::runtime_error( "Error in sending 'get error' command" ) );
    return 9999;
  }
  
  usleep(100000);
  
  std::string sr = serialRead();
  
  std::stringstream s;
  s << sr;
  s >> error;

  return error;
  
}

// +------------------------------------------------------+
// | get number of elements in parameter stack            |
// +------------------------------------------------------+

int Pollux::getNStack( void ) throw( std::runtime_error ) {

  std::lock_guard<std::mutex> _( polluxMut() );

  int ns = 0;

  tcflush( _fd, TCIOFLUSH );
  
  std::string ngsp = "ngsp";

  int status = write( _fd, ngsp.c_str(), ngsp.size() );
  if( status != ngsp.size() ) {
    throw( std::runtime_error( "Error in sending 'ngsp' command" ) );
    return 9999;
  }
  
  usleep(100000);
  
  std::string sr = serialRead();
    
  usleep(100000);
  
  std::stringstream s;
  //s << _buff;
  s << sr;
  s >> ns;

  return ns;
  
}

// +------------------------------------------------------+
// | check for  status code = 0 (or = 1)                  |
// +------------------------------------------------------+

void Pollux::checkCompletion( int axis ) throw( std::runtime_error ) {

  std::lock_guard<std::mutex> _( polluxMut() );

  if( axis < 1 || axis > 2 ) {
    throw( std::runtime_error( "Axis must be '1' or '2'" ) );
    return;
  }
  
  if( ! _connected[axis-1] ) {
    try {
      this->connectToDevice( axis );
    } catch( std::runtime_error &e ) {
      std::cout << e.what() << std::endl;
      return;
    }
  }

  std::stringstream nstatus;
  nstatus << axis << " nstatus ";

  int status = 0;

  if( _writeDots ) std::cout << "Waiting for motor ready";
  
  std::string sr;
  do {
    tcflush( _fd, TCIOFLUSH );
    if( _writeDots ) std::cout << "." << std::flush;
    status = write( _fd, nstatus.str().c_str(), nstatus.str().size() );
    if( status != nstatus.str().size() ) {
      if( _writeDots ) std::cout << std::endl << std::flush;
      throw( std::runtime_error( "Error in sending 'get status' command" ) );
      return;
    }
    usleep(100000);
    sr = serialRead();
    if( sr.substr(0,1) != "0" && sr.substr(0,1) != "1" ) {
      printf( "======> %s\n", sr.c_str() );
      if( _writeDots ) std::cout << std::endl << std::flush;
      std::stringstream s;
      s << "Error in getting device status: " << _buff[0];
      throw( std::runtime_error( s.str() ) );
    } 
    usleep(100000);
  } while( sr.substr(0,1) != "0" );

  if( _writeDots ) std::cout << std::endl << std::flush;

  return;

}

// +------------------------------------------------------+
// | get the motor error meaning                          |
// +------------------------------------------------------+

std::string Pollux::getErrorString( int error ) {

  std::string es;
  switch( error ) {
  case 1002:
    es = "Parameter stack underrun";
    break;
  case 1003:
    es = "Parameter out of range";
    break;
  case 1004:
    es = "Position range exceeded";
    break;
  case 1009:
    es = "Parameter stack lacking space (<10 parameters left)";
    break;
  case 1010:
    es = "RS-232 input buffer lacking space (<30 characters left)";
    break;
  case 1015:
    es = "Limit setting inconsistent";
    break;
  case 1100:
    es = "Limit switches states inconsistent / both active";
    break;
  case 2000:
    es = "Unknown command";
    break;
  default:
    es = "Unknown error";
    break;
  }

  return es;
  
}

// +------------+
// | serialRead |
// +------------+

std::string Pollux::serialRead( void ) {

  std::lock_guard<std::mutex> _( polluxMutSR() );

  fd_set read_fds, write_fds, except_fds;
  FD_ZERO( &read_fds );
  FD_ZERO( &write_fds );
  FD_ZERO( &except_fds );
  FD_SET( _fd, &read_fds );
  struct timeval timeout;

  timeout.tv_sec = 1;        // timeout: 1 s.
  timeout.tv_usec = 0;

  std::string s = "";
  char buff[512];
  int status = 0;
  int rv = 0;
  do {
    if( ( rv = select( _fd+1, &read_fds, &write_fds, &except_fds, &timeout )) ==1 ) {
      status = read( _fd, buff, 1 );
      if( status == 1 ) {
	s += buff[0];
      }
    } 
  } while( status > 0 && rv == 1 );
  return s;
}

