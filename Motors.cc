// +--------------------------------------------------------+
// | To move motors                                         |
// | Benigno Gobbo                                          |
// | (c) INFN Sezione di Trieste                            |
// | V1.0                                                   |
// | 16 July 2015                                           |
// +--------------------------------------------------------+

#include <string>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <dirent.h>
#include <unistd.h>
#include <pthread.h>
#include "pollux/Pollux.h"
#include "orientalmotors/OM.h"
#include "constants.h"
#include "Motors.h"

Motors::Motors( void ) {
  _motorDevicesFound = searchMotorDevices();
  _initialized = false;
}

bool Motors::searchMotorDevices( void ) {

  char buff[256];
  std::string OgrepX = "\'ATTRS{serial}==\"AE018HGJ\"\'";
  std::string OgrepY = "\'ATTRS{serial}==\"AE018HGL\"\'";
  //std::string Pgrep = "\'ATTRS{idProduct}==\"2303\"\'";
  //std::string Pgrep = "\'ATTRS{serial}==\"FTVQJWGZ\"\'";
  std::string Pgrep = "\'ATTRS{serial}==\"FTVNNFJ9\"\'";
  std::string comm1 = "udevadm info -a -n ";
  std::string comm2 = " | grep ";

  // Look for devices:
  std::vector<std::string> files;
  DIR *dp = opendir( "/dev" );
  struct dirent *dirp;
  while( (dirp = readdir(dp))  != NULL ) {
    files.push_back( std::string( dirp->d_name ) );
  }
  std::string s = "ttyUSB";
  std::vector<std::string> devices;

  for( unsigned int i=0; i<files.size(); i++ ) {
    if( files[i].substr(0,6) == s ) {
      devices.push_back( "/dev/" + files[i] );
    }
  }
  
  // Oriental Motors
  _omDevices.resize(2);
  for( unsigned int i=0; i<devices.size(); i++ ) {
    std::string command = comm1 + devices[i] + comm2 + OgrepX;
    FILE* f = popen( command.c_str(), "r" );
    if( f ) {
      while( !feof( f ) ) {
	if( fgets( buff, 256, f ) != NULL ) {
	  _omDevices[0] = devices[i];
	}
      }
      pclose( f );
    }
    command = comm1 + devices[i] + comm2 + OgrepY;
    f = popen( command.c_str(), "r" );
    if( f ) {
      while( !feof( f ) ) {
	if( fgets( buff, 256, f ) != NULL ) {
	  _omDevices[1] = devices[i];
	}
      }
      pclose( f );
    }
  }

  // PImicos
  for( unsigned int i=0; i<devices.size(); i++ ) {
    std::string command = comm1 + devices[i] + comm2 + Pgrep;
    FILE *f = popen( command.c_str(), "r" );
    if( f ) {
      while( !feof( f ) ) {
	if( fgets( buff, 256, f ) != NULL ) {
	  _pxDevices.push_back( devices[i] );
	}
      }
      pclose( f );
    }
  }
  
  if( _pxDevices.size() == 1 && _omDevices.size() == 2 ) {
    return( true );
  }
  else {
    return( false );
  }
}

void Motors::initializeMotors( void ) {

  if( ! _motorDevicesFound || _initialized ) return;

  // PImicos init.
  try {
    _px =  new Pollux( _pxDevices[0] );  
    _px->connectToPort();
    std::cout << "\033[7;32m Pollux Motors initialization...\033[0m"  << std::endl;
    for( int axis=1; axis<3; axis++ ) {
      _px->connectToDevice( axis );
      _px->clear( axis );
      std::cout << "\033[34m Motor " << axis << " calibration... \033[0m"  << std::endl;
      _px->calibrate( axis );
      std::cout << "\033[34m Motor " << axis << " range measurement... \033[0m"  << std::endl;
      _px->rangeMeasure( axis );
    }
    std::cout << "\033[32m Pollux Motors initialization completed. \033[0m"  << std::endl; 
  } catch( std::runtime_error &e ) {
    std::cout << "\033[31m Pollux Motors initialization failed. \033[0m"  << std::endl; 
    _initialized = false;
    return;
  }

  // Oriental Motor init. 
  std::string command, answer;
  try {

    std::cout << "\033[7;32m Oriental Motors initialization (be patient, it takes a while)...\033[0m"  << std::endl;

    _om[0] = new OM( _omDevices[0] );
    _om[0]->connectToDevice();

    _om[1] = new OM( _omDevices[1] );
    _om[1]->connectToDevice();

    _om[0]->init( MC::omMaxX );
    _om[1]->init( MC::omMaxY );

    std::cout << "\033[34m Waiting motors end moving home... \033[0m" << std::endl;
    
    std::string sx="", sy="";
    do {
      command = "SIGMOVE\n";
      answer = _om[0]->sendCommandToMotor( command );
      sx = answer.substr( answer.find( "=" )+1, std::string::npos );
      answer = _om[1]->sendCommandToMotor( command );
      sy = answer.substr( answer.find( "=" )+1, std::string::npos );
      double omXpos = updateMotorPosition( 0 );
      double omYpos = updateMotorPosition( 1 );
      std::cout << "\033[0m Oriental Motors position. X:\033[32m " << omXpos 
		<< "\033[0m, Y:\033[32m " << omYpos << "\033[0m" << std::endl;
      sleep( 1 );
    } while( sx == "1" || sy == "1" );

    std::cout << "\033[34m Oriental Motors initialization completed. \033[0m"  << std::endl;

  } catch( std::runtime_error &e ) {
    std::cout << "\033[31m Oriental Motors initialization failed. \033[0m"  << std::endl;
    _initialized = false;
    return;
  }

  _initialized = true;
  return;
  
}

double Motors::updateMotorPosition( int n ) {

  int type = -1, axis = -1;
  if( n >= 0 ) {
    if( n < 2 ) {
      type = 0;
      axis = n;
    }
    else if( n < 4 ) {
      type =1;
      axis = n-2;
    }
  }

  if( type < 0 || type > 1 || axis < 0 || axis > 1 ) return( -1. );
  
  double pos = 0;
  if( type == 0 ) {
    pos = _om[ axis ]->getMotorPosition();
  }
  else if ( type == 1 ) {
    pos = _px->getPosition2( 1+axis );
  }
  return( pos );
}

double Motors::moveMotorAbs( int n, double p ) {

  int type = -1, axis = -1;
  if( n >= 0 ) {
    if( n < 2 ) {
      type = 0;
      axis = n;
    }
    else if( n < 4 ) {
      type =1;
      axis = n-2;
    }
  }

  if( type < 0 || type > 1 || axis < 0 || axis > 1 ) return( -1. );

  double pos = 0;
  if( type == 0 ) {
    pos = _om[ axis ] ->moveAbs( int(p) );
  }
  else if ( type == 1 ) {
    _px->goToAbsolutePosition( 1+axis, p );
    pos = _px->getPosition2( 1+axis );
  }
  return( pos );
}

double Motors::moveMotorInc( int n, double i ) {

  int type = -1, axis = -1;
  if( n >= 0 ) {
    if( n < 2 ) {
      type = 0;
      axis = n;
    }
    else if( n < 4 ) {
      type =1;
      axis = n-2;
    }
  }

  double pos = 0;
  if( type == 0 ) {
    pos = _om[ axis ]->moveInc( int(i) );
  }
  else if ( type == 1 ) {
    _px->goToRelativePosition2( 1+axis, i );
    pos = _px->getPosition2( 1+axis );
  }
  return( pos );
}

void Motors::stopMotors( void ) {

  std::cout << "\033[31m Stopping motors... \033[0m" << std::endl;

  std::string command = "MSTOP\n";
  std::string answer = _om[0]->sendCommandToMotor( command );
  std::cout << "om X: " << answer << std::endl;
  answer = _om[1]->sendCommandToMotor( command );
  std::cout << "om Y: " << answer << std::endl;
  
  _px->stopMotion( 1 );
  std::cout << "px X stopped " << std::endl;
  _px->stopMotion( 2 );
  std::cout << "px Y stopped " << std::endl;
}

void Motors::resetMotor( int n ) {
  
  int type = -1, axis = -1;
  if( n >= 0 ) {
    if( n < 2 ) {
      type = 0;
      axis = n;
    }
    else if( n < 4 ) {
      type =1;
      axis = n-2;
    }
  }

  if( type == 0 ) {
    _om[ axis ]->reset();
  }
  else if ( type == 1 ) {
    _px->reset( 1+axis );
  }
}
