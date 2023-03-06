#pragma once

#include <cmath>

#include "Shape.h"

class Triangle : public Shape
{
private:
	double e1, e2, e3, radius;
//	int sym_angle;
	Polygon poly;

public:
	Triangle();

	Polygon get_poly();
	void update(State3D pos);
	double getRadius();
//	int get_symangle();
	bool intersect(Square sq);
	bool intersect_robot(Robot* robot);
	bool intersect_wall(Wall* wall);
};
