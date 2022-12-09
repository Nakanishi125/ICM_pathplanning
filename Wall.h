#pragma once

#include "Square.h"

class Wall
{
private:
	MultiSquare wall;

public:
	Wall();
	bool intersect(Square obj);
	bool intersect(MultiSquare objs);
	MultiSquare getter() { return wall; }
};

