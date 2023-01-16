#include "TShape.h"
#include "Square.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>


namespace bp = boost::property_tree;

TShape::TShape()
	:long_side(150), short_side(100), long_pro(40), short_pro(40), sym_angle(360)
{
}


Polygon TShape::get_poly()
{
	return poly;
}

double TShape::getRadius()
{
	return sqrt(long_side * long_side + short_side * short_side);
}


int TShape::get_symangle()
{
	return sym_angle;
}


void TShape::update(State3D pos)
{
	double radAngle = deg_to_rad(pos.th);
	double aspect = atan2(long_side, short_side);

	Point2D refPoint(pos.x - 0.5 * std::sqrt(short_side * short_side + long_side * long_side) * std::cos(radAngle - aspect),
		             pos.y + 0.5 * std::sqrt(short_side * short_side + long_side * long_side) * std::sin(aspect + radAngle));
	Point2D p2(refPoint.x + short_side * std::cos(radAngle), refPoint.y + short_side * std::sin(radAngle));
	Point2D p3(p2.x + long_pro * std::sin(radAngle), p2.y - long_pro * std::cos(radAngle));
	Point2D p4(p3.x - ((short_side - short_pro)/2.0) * std::cos(radAngle), p3.y - ((short_side - short_pro)/2.0) * std::sin(radAngle));
	Point2D p5(p4.x + (long_side - long_pro) * std::sin(radAngle), p4.y - (long_side - long_pro) * std::cos(radAngle));
	Point2D p6(p5.x - short_pro * std::cos(radAngle), p5.y - short_pro * std::sin(radAngle));
	Point2D p7(p6.x - (long_side - long_pro) * std::sin(radAngle), p6.y + (long_side - long_pro) * std::cos(radAngle));
	Point2D p8(p7.x - ((short_side - short_pro)/2.0) * std::cos(radAngle), p7.y - ((short_side - short_pro)/2.0) * std::sin(radAngle));

	std::vector<Point2D> vert = { refPoint, p2, p3, p4, p5, p6, p7, p8};
	poly = Polygon(vert);
}


MultiSquare TShape::get_square()
{
	Square sq1(poly.get(0), poly.get(1), poly.get(2), poly.get(7));
	Square sq2(poly.get(3), poly.get(4), poly.get(5), poly.get(6));
	std::vector<Square> msq = { sq1, sq2 };

	return MultiSquare(msq);
}


bool TShape::intersect(Square sq)
{
	MultiSquare msq = get_square();
	for(int i=0; i<(int)msq.size(); ++i){
		if(sq.intersect(msq.element(i)))	return true;
	}
	return false;
}

bool TShape::intersect_robot(Robot* robot)
{
	return robot->intersect(get_square());
}


bool TShape::intersect_wall(Wall* wall)
{
	return wall->intersect(get_square());
}
