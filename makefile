CC=gcc
CXX=g++
CFLAGS=-I. -I//usr/local/include/hokuyoaist-3/ -I//usr/local/include/flexiport-2
CXXFLAGS=$(CFLAGS)
LDFLAGS=-L/usr/local/lib/gearbox 
LDLIBS=-lflexiport -lhokuyoaist


all: URGserver 

turn_to: URGclient.o turn_to.o 
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS) 

URGserver:socketUtil.o URGserver.o
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS)

URGclient: socketUtil.o URGclient.o
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS)

URGquit: socketUtil.o URGquit.o
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS) -lMage -lm -lpthread

clean:
	rm -f *.o *~ URGserver URGclient URGquit
