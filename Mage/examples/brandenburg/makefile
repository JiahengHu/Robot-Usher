CC = gcc
LIBDIR = ../../lib/

all: brand

brand: brand.o $(LIBDIR)libMage.a
	gcc $(CFLAGS) -o brand brand.o $(LIBDIR)libMage.a -lm -lpthread

clean:
	rm -f *.o *~

