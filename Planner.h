#pragma once

#include "Node.h"
#include "icmMath.h"

class Planner{
public:
	virtual NodeList plan(Node ini, Node fin, State3D goal) = 0;

	virtual bool robot_update(Node newnode);


protected:
	Planner(){}
};
