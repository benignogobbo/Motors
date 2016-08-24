#
# To build programs...
# Author: Benigno
#

CC     = g++
CFLAGS = -O -std=c++11

all: libOM.a

libOM.a:	OM.o
	ar rvu libOM.a OM.o

OM.o: OM.cc OM.h
	$(CC) $(CFLAGS) -c OM.cc

clean: 
	rm -f *.o libOM.a *~ 





