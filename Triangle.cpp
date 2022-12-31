
#include "Triangle.h"
#include "icmMath.h"

Triangle::Triangle()
	:e1(120), e2(120), e3(120), sym_angle(120), radius()
{
	double under = 2*sqrt(1 - ( (e2*e2+e3*e3-e1*e1)/(2*e2*e3) * (e2*e2+e3*e3-e1*e1)/(2*e2*e3)));
	radius = e1/under;
}


void Triangle::update(State3D pos)
{
	double radAngle = deg_to_rad(pos.th);	
	double alpha = std::acos(e1/(2*this->radius));
	double beta = std::acos(e3/(2*this->radius));

	Point2D refPoint(pos.x - 0.5*this->radius*cos(alpha + radAngle),
					 pos.y - 0.5*this->radius*sin(alpha + radAngle));
	Point2D p2(refPoint.x + e1*cos(radAngle), refPoint.y + e1*sin(radAngle));
	Point2D p3(refPoint.x + e3*cos(radAngle + alpha + beta),
			   refPoint.y + e3*sin(radAngle + alpha + beta));

	std::vector<Point2D> vert = {refPoint, p2, p3};
	poly = Polygon(vert);
}


double Triangle::getRadius()
{
	return radius;
}


int Triangle::get_symangle()
{
	return sym_angle;
}



