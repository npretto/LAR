CXX=g++
CXXFLAGS=`pkg-config --cflags tesseract opencv` -std=c++11
LDLIBS=`pkg-config --libs tesseract opencv`

SRCS:=$(wildcard src/*.cpp)
OBJS:=$(patsubst %.cpp,%.o,$(SRCS))

all: $(OBJS)

src/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf src/*.o
	
.PHONY: all clean
