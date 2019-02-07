
CXX=g++
CXXFLAGS=`pkg-config --cflags tesseract opencv` -std=c++11 -O2
LDLIBS=`pkg-config --libs tesseract opencv` -std=c++11 -O2

SRCS:=$(wildcard src/*.cpp)
OBJS:=$(patsubst %.cpp,%.o,$(SRCS))

all: $(OBJS) main.out

src/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@


main.out: $(OBJS) 
	$(CXX)  $(OBJS) $(LDLIBS) main.cpp -o main.out

clean:
	rm -rf src/*.o
	rm -f main.out
	rm -f main.o
	
.PHONY: all clean
