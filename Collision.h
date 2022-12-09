#pragma once

#include "Shape.h"
#include "Robot.h"
#include "Wall.h"

class Collision
{
public:
	static bool RW_intersect();
	static bool RS_intersect(Shape* shape);
	static bool WS_intersect(Shape* shape);
	static bool LS_intersect(Shape* shape, Link link);
	static bool RL_intersect(Link link);
};

