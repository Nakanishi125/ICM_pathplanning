#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

#include "Controller.h"
#include "Rectangle.h"
#include "LShape.h"
#include "TShape.h"

namespace bp = boost::property_tree;
Controller* Controller::instance = nullptr;


Shape* Controller::shape_create()
{
	bp::ptree pt;
	read_ini("config/ObjectParameter.ini", pt);

	boost::optional<int> carrier = pt.get_optional<int>("target.shape");
	int sh = carrier.get();
	if(sh == 1)	return new Rectangle;
	if(sh == 2) return new LShape;
	if(sh == 4) return new TShape;

	assert(true);
	return 0;
}


Controller::Controller()
	:robot(), shape(), wall()
{
	robot = new Robot();
	shape = shape_create();
	wall = new Wall();
}


Controller::~Controller()
{
	delete robot;
	delete wall;
}


Controller* Controller::get_instance()
{
	if(instance == nullptr){
		instance = new Controller();
	}
	return instance;
}


void Controller::robot_update(Node newnode)
{
	robot->update(newnode);
}

void Controller::shape_update(State3D st)
{
	shape->update(st);
}


bool Controller::RintersectR(Node newnode)
{
	robot->update(newnode);
	return robot->rr_intersect();
}


bool Controller::RintersectW(Node newnode)
{
	robot->update(newnode);
	if(robot->intersect(wall->getter()))return true;
	return false;
}


bool Controller::RintersectS(Node newnode, State3D st)
{
	robot->update(newnode);
	shape->update(st);
	if(robot->intersect(shape->get_square()))return true;
	return false;
}

bool Controller::RintersectS()
{
	return robot->intersect(shape->get_square());
}

bool Controller::WintersectS()
{
	return wall->intersect(shape->get_square());
}


