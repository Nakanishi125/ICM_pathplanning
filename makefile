TARGET = Manipulation

SRCS =  main.cpp 
SRCS += icmMath.cpp CSpace.cpp RRT.cpp CFree.cpp CFreeICS.cpp
SRCS += Labeling.cpp Link.cpp Node.cpp OneHand.cpp PointCloud.cpp Rectangle.cpp Planner.cpp FormClosure.cpp PSO.cpp
SRCS += Robot.cpp RRTTree.cpp Shape.cpp Square.cpp Wall.cpp LShape.cpp TaskSet.cpp Problem.cpp Controller.cpp pathsmooth.cpp TShape.cpp
OBJS = $(SRCS:.cpp=.o)

CXX = g++
#CXXFLAGS = -g -Wall 
CXXFLAGS = -flto -Wall -O3 -mtune=native -march=native -mfpmath=both
INCDIR = -I/usr/include

#LIBDIR = -L/usr/local/lib
#LIBS = -lompl -lmlpack

all: $(TARGET)

.cpp.o:
	$(CXX) $(CXXFLAGS) -o $@ -c $<

$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LIBDIR) $(LIBS)

clean:
	rm -f *.o

depend:
	makedepend $(INCDIR) $(SRCS)
