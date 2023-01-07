#pragma once

#include "Square.h"

class Polygon
{
private:
	std::vector<Point2D> vert;

public:
	Polygon();
	Polygon(std::vector<Point2D> vert);

	std::vector<Point2D> getter();
	Point2D get(int i);
	void add(Point2D pt);
};


class Shape
{
public:
	virtual Polygon get_poly() = 0;
	virtual void update(State3D pos) = 0;
	virtual double getRadius() = 0;
//	virtual int get_symangle() = 0;
	virtual MultiSquare get_square() = 0;
};

