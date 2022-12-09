#pragma once

#include <cmath>
#include <array>

#include "Shape.h"


class LShape : public Shape
{
private:
	double long_side, short_side, long_pro, short_pro;
	int sym_angle;
	Polygon poly;

public:
	LShape();

	Polygon get_poly();
	void update(State3D pos);
	double getRadius();
	int get_symangle();
	MultiSquare get_square();

};