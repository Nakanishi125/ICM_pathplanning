#include "Collision.h"



bool Collision::RS_intersect(Shape* shape)
{
	Robot* robot = Robot::get_instance();
	//Shape* shape = Shape::get_instance();
	
	if (robot->intersect(shape->get_square()))	return true;
	return false;
}


bool Collision::WS_intersect(Shape* shape)
{
	Wall* wall = Wall::get_instance();
	//Shape* shape = Shape::get_instance();

	if (wall->intersect(shape->get_square()))	return true;
	return false;
}


bool Collision::RW_intersect()
{
	Robot* myRobo = Robot::get_instance();
	Wall* wall = Wall::get_instance();

	if (myRobo->intersect(wall->getter()))	return true;
	return false;
}


bool Collision::LS_intersect(Shape* shape, Link link)
{
	if (link.intersect(shape->get_square()))	return true;
	return false;
}


bool Collision::RL_intersect(Link link)
{
	Robot* robot = Robot::get_instance();
	//Shape* shape = Shape::get_instance();

	if (robot->intersect(link.get_square()))	return true;
	return false;
}
