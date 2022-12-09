
#include "Planner.h"
#include "Controller.h"


bool Planner::robot_update(Node newnode)
{
	Controller* controller = Controller::get_instance();
	controller->robot_update(newnode);

	if (controller->RintersectR(newnode)) {
		return false;
	}
	if (controller->RintersectW(newnode)) {
		return false;
	}
	return true;
}

