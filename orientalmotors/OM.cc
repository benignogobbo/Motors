// +--------------------------------------------------------+
// | A Oriental Motor EZS II motor control library          |
// | via RS232 interface.                                   |
// |                                                        |
// | Benigno Gobbo                                          |
// | (c) INFN Sezione di Trieste                            |
// | V1.0                                                   |
// | 15 October 2013                                        |
// +--------------------------------------------------------+

#include <mutex>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <string>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdio>
#include <sys/time.h>
#include <sstream>
#include <termios.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "OM.h"

//bool OM::_motorsCommunicationLocked = false;
std::mutex & mutexOM() {
  static std::mutex m;
  return m;
}

// +-------------+
// | to_string   |
// +-------------+
// This function should be availabe in C++11, that is not
// default in gcc 4.8.2
namespace patch {
  template <typename T > std::string to_string( const T& n ) {
    std::ostringstream stm;
    stm << n;
    return stm.str();
  }
}
  
// +-------------+
// | constructor |
// +-------------+

OM::OM( std::string device ) {
  _device = device;
  _fd = 0;
}

// +-----------------+
// | connectToDevice |
// +-----------------+

void OM::connectToDevice( void ) throw( std::runtime_error ) {

  _fd = open( _device.c_str(), O_RDWR | O_NOCTTY );
  if( _fd < 0 ) {
    std::stringstream s;
    s << strerror( errno );
    throw( std::runtime_error( s.str() ) );
    return;
  }  

  tcgetattr( _fd, &_oldtio );
  _newtio = _oldtio;

  cfsetispeed( &_newtio, (speed_t)B9600 );
  cfsetospeed( &_newtio, (speed_t)B9600 );
  cfmakeraw(   &_newtio );

  tcsetattr( _fd, TCSANOW, &_newtio );  
  tcflush( _fd, TCIOFLUSH );

  bool found = false;
  char label = 0;

  for( int i=0; i<2 && !found; i++ ) {
    char c = i + '0';
    for( int j=0; j<2; j++ ) {
      std::string command;
      command = "@"; command += c; command +=  "\n";
      int status = write( _fd, command.c_str(), command.size() );
      if( status != command.size() ) {
	std::stringstream s;
	s << strerror( errno ); 
	throw ( std::runtime_error( s.str() ) );
      } 

      std::string sbuff = serialRead();
      std::string sfind = ""; sfind += c; sfind += ">";
      int pos = sbuff.find( sfind );
      if( pos != std::string::npos ) {
	found = true;
	label = c;
      }
    }
  }

  if( found ) {
    std::string command = "@"; command += label; command += "ver\n";
    int status = write( _fd, command.c_str(), command.size() );
    if( status != command.size() ) {
      std::stringstream s;
      s << strerror( errno ); 
      throw ( std::runtime_error( s.str() ) );
    } 

    std::string s = serialRead();
    if( s.size() > 0 ) {
      //std::cout << "\033[32m Found motor labelled '" << label << "'\033[0m" << std::endl;
      std::size_t found;
      s = s.substr( 0,  s.rfind(">")-3 );
      found = s.rfind( "\r\n" );
      if( found != std::string::npos ) {
	s = s.substr( found+2,  std::string::npos );
      }
      //std::cout << "Firmware Version: " << s << std::endl;
    }
  }
  else {
    std::stringstream s;
    s << "\033[31m No motors responding\033[0m"; 
    throw ( std::runtime_error( s.str() ) );
  }
  //_id = label;
}

// +----------------+
// | restore Device |
// +----------------+

void OM::restoreDevice( void ) throw( std::runtime_error ) {

  tcsetattr( _fd, TCSANOW, &_oldtio ); /* restore old port settings */
  tcflush( _fd, TCIOFLUSH );
  if( close( _fd ) < 0 ) {
    std::stringstream s;
    s << strerror( errno ); 
    throw ( std::runtime_error( s.str() ) );
  } 

  return;
}

// +--------------------+
// | sendCommandToMotor |
// +--------------------+

std::string OM::sendCommandToMotor( std::string command ) throw( std::runtime_error ) {

  //waitForUnlock(); 
  std::lock_guard<std::mutex> _( mutexOM() );

  tcflush( _fd, TCOFLUSH );
  //command = "@" + _id + command;
  int status = write( _fd, command.c_str(), command.size() );
  if( status != command.size() ) {
    throw( std::runtime_error( "Error in sending command to motor" ) );
    //unlock();
    return( "" );
  }
  std::string s = getResponseFromMotor();
  //unlock();
  return( s );
}

// +----------------------+
// | getResponseFromMotor |
// +----------------------+
std::string OM::getResponseFromMotor( void ) throw( std::runtime_error ) {

  std::string s = serialRead();

  // Try to remove leading echo, if any...
  std::size_t found;
  found = s.rfind( ">" );
  if( found > 0 ) {
    s = s.substr( 0,  s.rfind(">")-3 );
  }
  else {
    s = "";
  }
  //
  //if( found != std::string::npos ) {
  //  s = s.substr( found+2,  std::string::npos );
  //}

  return s;

}

// +------------------+
// | getMotorPosition |
// +------------------+
double OM::getMotorPosition( void ) throw( std::runtime_error ) {

  double pos;
  usleep(200000);
  std::string command = "PF\n";
  std::string answer = sendCommandToMotor( command );
  std::stringstream s;
  s << answer.substr( 4, answer.find( "mm" ) - 5 );
  s >> pos;
  return( pos );
}

// +--------+
// | goHome |
// +--------+
void OM::goHome( bool noWait ) throw( std::runtime_error ) {

  //waitForUnlock();
  std::string command, answer;

  command = "MGHP\n";
  answer = sendCommandToMotor( command );

  if( noWait ) {
    //unlock();
    return;
  }

  float pos = 1;
  bool wentBelowZero = false;
  while( pos != 0 || !wentBelowZero ) {
    usleep(200000);
    pos = getMotorPosition();
    if( pos < 0 ) wentBelowZero = true; 
  }
  //unlock();
  return;
}

// +--------------+
// | selectDevice |
// +--------------+
//void OM::selectDevice( std::string id ) throw( std::runtime_error ) {
//  std::string command = "@"+id+"\n";
//  std::string answer = sendCommandToMotor( command );
//  _id = id;
//  return;
//}

// +-------+
// | setID |
// +-------+
//void OM::setID( std::string id ) throw( std::runtime_error ) {
//  _id = id;
//  std::string command = "ID="+_id+"\n";
//  std::string answer = sendCommandToMotor( command );
//  selectDevice( id );
//  return;
//}

// +------------+
// | getVersion |
// +------------+
std::string OM::getVersion( void ) throw( std::runtime_error ) {

  //waitForUnlock();
  std::string command = "VER\n";
  std::string answer = sendCommandToMotor( command );
  //unlock();
  return( answer );
}

// +------+
// | init |
// +------+
void OM::init( int limp ) throw( std::runtime_error ) {

  //waitForUnlock();
  std::string command = "", answer = "";
  command = "HOMETYP=12; ECHO=0; LIMP=" + patch::to_string( limp ) + "; LIMN=0; SLACT=1; VS=1; VR=100; TA=3; TD=3\n";
  answer = sendCommandToMotor( command );
  
  goHome( true );

  //unlock();
  return;

}

// +---------+
// | moveAbs |
// +---------+
double OM::moveAbs( double abs, bool noWait ) throw( std::runtime_error ) {

  //waitForUnlock();
  std::stringstream s;
  s << "MA " << abs << "\n";

  std::string answer = sendCommandToMotor( s.str() );

  if( noWait ) {
    //unlock();
    return( 0 );
  }

  double pos = -1;
  while( int(pos) != abs ) {
    usleep(200000);
    pos = getMotorPosition();
  }

  //unlock();
  return( pos );

}

// +---------+
// | moveInc |
// +---------+
double OM::moveInc( double inc, bool noWait ) throw( std::runtime_error ) {

  //waitForUnlock();
  double oldpos = getMotorPosition();
  std::stringstream s;
  s << "DIS=" << inc << " ; MI\n";
  std::string answer = sendCommandToMotor( s.str() );

  if( noWait ) {
    //unlock();
    return( 0 );
  }

  double pos = -1;
  while( int(pos) != inc+oldpos ) {
    usleep(200000);
    pos = getMotorPosition();
  }

  //unlock();
  return( pos );
}

// +------------+
// | serialRead |
// +------------+

std::string  OM::serialRead( void ) {

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

// +-------+
// | reset |
// +-------+
std::string OM::reset( void ) throw( std::runtime_error ) {

  usleep(200000);
  std::string command = "RESET\n";
  std::string answer = sendCommandToMotor( command );
  return( answer );
}

//void OM::waitForUnlock( void ) {
//  while( _motorsCommunicationLocked == true ) {
//    usleep( 100000 );
//  }
//  _motorsCommunicationLocked = true;
//}
