
#include "Planner.h"
#include "Node.h"
#include "icmMath.h"

class Problem{
private:
	Node initial, finish;
	State3D goal;

	Planner* solver;

public:
	Problem(Planner* solver);
	NodeList pathplanning();
};


Node read_initial();
Node read_finish();
State3D read_goal();
