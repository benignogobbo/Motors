#
# To build programs...
# Author: Benigno
#

CC     = g++
CFLAGS += -O -std=c++11

all:	libPollux.a

libPollux.a:	Pollux.o
	ar rvu libPollux.a Pollux.o

Pollux.o:	Pollux.cc Pollux.h
	$(CC) $(CFLAGS) -c Pollux.cc

clean: 
	rm -f *.o libPollux.a *~




