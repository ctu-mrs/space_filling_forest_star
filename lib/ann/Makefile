
CXX=g++
CXXINCLUDE+=-I/usr/local/include
LDFLAGS+=-L/usr/local/lib -lpthread -lm
TARGET=ann_test

LDFLAGS+=-Lann/lib -lANN

SRC=$(wildcard *.cc)
OBJS=$(patsubst %.cc,%.o,$(wildcard *.cc))

all: $(OBJS) lib_dir mpnn
	$(CXX) $(OBJS) $(LDFLAGS) -o $(TARGET)

lib_dir:
	mkdir -p ann/lib/

mpnn:
	$(MAKE) -C ann/src linux-g++

.cc.o:
	$(CXX) $(CXXINCLUDE) -c $<


clean:
	rm -f $(OBJS) $(TARGET) $(TARGET).core
	$(MAKE) -C ann/src clean


