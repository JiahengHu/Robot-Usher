CC = gcc
LIBDIR = ../../lib/

all: facetest

test: test.o videoRoutines.o ppmIO.o face.o visionRoutines.o $(LIBDIR)libMage.a
	gcc $(CFLAGS) -o test test.o videoRoutines.o ppmIO.o visionRoutines.o face.o $(LIBDIR)libMage.a -lm -lpthread

facetest: test2.o videoRoutines.o ppmIO.o face.o visionRoutines.o $(LIBDIR)libMage.a
	gcc $(CFLAGS) -o facetest test2.o videoRoutines.o ppmIO.o visionRoutines.o face.o $(LIBDIR)libMage.a -lm -lpthread

clean:
	rm -f *.o *~

