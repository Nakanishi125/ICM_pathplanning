#include "Wall.h"


Wall::Wall()
	:wall()
{
	Point2D w1(-300, -5.001);
	Point2D w2(-300, -0.001);
	Point2D w3(300, -0.001);
	Point2D w4(300, -5.001);
	Square wall1(w1, w2, w3, w4);
	wall.push(wall1);
}


bool Wall::intersect(Square obj)
{
	return wall.intersect(obj);
}

bool Wall::intersect(MultiSquare objs)
{
	for (int i = 0; i < (int)objs.size(); ++i) {
		if (wall.intersect(objs.element(i)))	return true;
	}

	return false;
}
