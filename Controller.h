#pragma once

#include "Shape.h"
#include "Robot.h"
#include "Wall.h"


class Controller
{
private:
	Robot* robot;
	Shape* shape;
	Wall* wall;

	Controller();
	~Controller();
	static Controller* instance;

	Shape* shape_create();
public:
	static Controller* get_instance();	

	void robot_update(Node newnode);
	void shape_update(State3D st);

// Below 4 functions can be used 
// when robot or shape has already updated properly
// by using above 2 functions
	bool RintersectR();
//	bool RintersectW();
	bool RintersectS();
	bool WintersectS();

// Below 4 functions are general version of above 4 func
	bool RintersectR(Node newnode);
	bool RintersectW(Node newnode);
	bool RintersectS(Node newnode, State3D st);
	bool WintersectS(State3D st);

	bool RintersectL(int index);
	bool LintersectS(int index);

};
