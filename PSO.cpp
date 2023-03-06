
#include "PSO.h"


double caging_func(Node node)
{
	State3D goal = read_goal();
	Controller* controller = Controller::get_instance();
	controller->robot_update(node);
	CFreeICS ics(node);

	if (controller->RintersectR(node)) {
		return 1000;
	}
	if (controller->RintersectW(node)) {
		return 1000;
	}
	
	std::vector<PointCloud> cfics = ics.extract();
	if(cfics.size() == 0)	return 1000;
	std::vector<PointCloud> cfree_objs;
	for(int i=0; i<(int)cfics.size(); ++i){
		for(int j=0; j<cfics[i].size(); ++j){
			if(cfics[i].get(j).y == goal.y &&
			   cfics[i].get(j).th == goal.th){
				cfree_objs.push_back(cfics[i]);
				break;
			}
		}
	}
	if(cfree_objs.size() != 1)	return 1000;
	PointCloud cfree_obj = cfree_objs[0];
	
	std::vector<int> x_list;
	for(int i=0; i<cfree_obj.size(); ++i){
		if(cfree_obj.get(i).y == goal.y && cfree_obj.get(i).th == goal.th){
			x_list.push_back(cfree_obj.get(i).x);
		}
	}
	std::sort(x_list.begin(), x_list.end());
	int len = (int)x_list.size();
	int index = len/2;

	int gx = x_list[index];
	goal.x = gx;

	int xmin = INT_MAX, xmax = INT_MIN;
	double max_dist = DBL_MIN;
	for(int i=0; i<cfree_obj.size(); ++i){
		double dist = calc_dist(cfree_obj.get(i), goal);
		int xtmp = cfree_obj.get(i).x;
        if (xtmp < xmin) xmin = xtmp;
        if (xtmp > xmax) xmax = xtmp;
	    int delta_x = (xmax - xmin) / 2;
	    dist = std::sqrt(dist * dist + delta_x * delta_x);
        if (max_dist < dist)   max_dist = dist;
	}

	return max_dist;
}
