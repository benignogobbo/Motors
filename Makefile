#
# To build programs...
# Author: Benigno
#

CC     = g++
CFLAGS = -O -std=c++11
DIRS   =  pollux orientalmotors 
LIBS   = -L./ -lMotors -L./pollux -lPollux -L./orientalmotors -lOM 

all: libs libMotors.a test

test:   test.cc libMotors.a ./orientalmotors/libOM.a ./pollux/libPollux.a
	$(CC) -o test test.cc $(LIBS) 

libs:
	for d in $(DIRS); do ( cd $$d; $(MAKE) ); done 

libMotors.a:    Motors.o constants.o
	ar rvu libMotors.a constants.o Motors.o

constants.o:	constants.cc constants.h
	$(CC) $(CFLAGS) -c constants.cc

Motors.o:	Motors.cc Motors.h
	$(CC) $(CFLAGS) -c Motors.cc

clean: 
	rm -f *.o *~
	for d in $(DIRS); do ( cd $$d; $(MAKE) clean ); done 





