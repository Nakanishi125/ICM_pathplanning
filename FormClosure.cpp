#include <iostream>

#include "CFreeICS.h"
#include "CFree.h"
#include "FormClosure.h"
#include "RRT.h"
#include "CFreeICS.h"
#include "Problem.h"
#include "Controller.h"
#include "TaskSet.h"
#include "Visual.h"

FormClosure::FormClosure(Node _ini)
	:ini(_ini), fcang(_ini), goal()
{
	goal = read_goal();
}


void FormClosure::init()
{
	Controller* controller = Controller::get_instance();
	controller->robot_update(ini);
	controller->shape_update(goal);	
}


int FormClosure::nind_to_lind(int nind)
{
	int index = -1;
	if (nind < 3)	index = nind + 1;
	else            index = nind + 2;

	return index;
}


int FormClosure::lind_to_nind(int lind)
{
	int nind = -1;
	if (lind < 4)	nind = lind - 1;
	else            nind = lind - 2;

	return nind;
}


void FormClosure::simple_narrow(std::vector<int> order)
{
	assert((int)order.size() == 6);
	for (int i = 0; i < (int)order.size(); ++i)	assert(i >= 0 && i < 6);
	const double mov_range = 0.1; 

	int next = 0;
	for (int i = 0; i < HalfNode::dof; ++i) {
		if (order[i] < next)	continue;
		next = explore(order[i]);
	}
	next = 0;
	for (int i = HalfNode::dof; i < (int)order.size(); ++i) {
		if (order[i] < next)	continue;
		next = explore(order[i]);
	}

}



int FormClosure::explore(int nind)
{
	Controller* controller = Controller::get_instance();
	const double mov_range = 0.1;

	while (fcang[nind] > -180) {
		controller->robot_update(fcang);

		if (controller->RintersectR()) {
			fcang[nind] += mov_range;
			if (fcang[nind] < -90)	fcang[nind] = -90;

			if (nind < HalfNode::dof) {
				for (int l = nind; l < 3; ++l) {
					int lind = nind_to_lind(l);
					if(controller->RintersectL(lind)){
						return l + 1;
					}
				}
				assert(false);
			}
			else {
				for (int r = nind; r < 6; ++r) {
					int lind = nind_to_lind(r);
					if(controller->RintersectL(lind)){
						return r + 1;
					}
				}
				assert(false);
			}
		}

		if(controller->RintersectS()){
			fcang[nind] += mov_range;
			if (fcang[nind] < -90)	fcang[nind] = -90;

			if (nind < HalfNode::dof) {
				for (int l = nind; l < 3; ++l) {
					int lind = nind_to_lind(l);
					if(controller->LintersectS(lind)){
						return l + 1;
					}
				}
				assert(false);
			}
			else {
				for (int r = nind; r < 6; ++r) {
					int lind = nind_to_lind(r);
					if(controller->LintersectS(lind)){
						return r + 1;
					}
				}
				assert(false);
			}
			assert(false);
		}

		else {
			fcang[nind] -= mov_range;
		}
	}

	fcang[nind] = ini.get_element(nind);
	return reverse_explore(nind);
}


int FormClosure::reverse_explore(int nind)
{
	Controller* controller = Controller::get_instance();
	const double mov_range = 0.1;

	while (fcang[nind] > 180) {
		controller->robot_update(fcang);
		if(controller->RintersectR()){
			fcang[nind] -= mov_range;
			if (fcang[nind] > 90)	fcang[nind] = 90;

			if (nind < HalfNode::dof) {
				for (int l = nind; l < 3; ++l) {
					int lind = nind_to_lind(l);
					if(controller->RintersectL(lind)){
						return l + 1;
					}
				}
				assert(false);
			}
			else {
				for (int r = nind; r < 6; ++r) {
					int lind = nind_to_lind(r);
					if(controller->RintersectL(lind)){
						return r + 1;
					}
				}
				assert(false);
			}
		}

		if(controller->RintersectS()){
			fcang[nind] -= mov_range;
			if (fcang[nind] > 90)	fcang[nind] = 90;

			if (nind < HalfNode::dof) {
				for (int l = nind; l < 3; ++l) {
					int lind = nind_to_lind(l);
					if(controller->LintersectS(lind)){
						return l + 1;
					}
				}
				assert(false);
			}
			else {
				for (int r = nind; r < 6; ++r) {
					int lind = nind_to_lind(r);
					if(controller->LintersectS(lind)){
						return r + 1;
					}
				}
				assert(false);
			}
		}

		else {
			fcang[nind] += mov_range;
		}
	}

	fcang[nind] = ini.get_element(nind);
	if (nind == 2 || nind == 5)	fcang[nind] = -90;
	return nind + 1;
}


void FormClosure::close()
{
	Controller* controller = Controller::get_instance();
	State3D goal = read_goal(); 
	std::vector<int> begin_left = { 0, 1, 2, 3, 4, 5 };
	std::vector<int> begin_right = { 3, 4, 5, 0, 1, 2 };
	double lcobj, rcobj = 0.0;
	std::vector<State3D> prev = { goal };
	PointCloud pc1(prev);

	// moving left hand first, and right hand
	init();
	CFreeICS* ics = new CFreeICS(ini);
	std::vector<PointCloud> cfree_ics = ics->extract();

	std::vector<PointCloud> cfree_objs;
	for(int i=0; i<cfree_ics.size(); ++i){
		for(int j=0; j<cfree_ics[i].size(); ++j){
			if(cfree_ics[i].get(j).y == goal.y &&
			   cfree_ics[i].get(j).th == goal.th){
				cfree_objs.push_back(cfree_ics[i]);
				break;
			}
		}
	}
	if(cfree_objs.size() != 1){
		std::cout << "The size of cfree_obj : " << cfree_objs.size() << std::endl;
		return;
	}
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
	
	double dobj = fc_eval(cfree_obj); 
	std::cout << "Before form closure: " << dobj << std::endl;
	delete ics;

	// moving left hand first, after that right hand
	std::vector<State3D> prev1 = { goal };
	PointCloud pc(prev1);
	controller->shape_update(goal);

	simple_narrow(begin_left);
	Node fcang_left = fcang;
	controller->robot_update(fcang_left);
	RasterCFO* raster = new RasterCFO();
	if(controller->RintersectR()){
		std::cout << "Robot intersect" << std::endl;
		lcobj = DBL_MAX;
	}
	std::vector<PointCloud> cfolr = raster->extract(pc, fcang_left);
	if(cfolr.size() != 1){
		std::cout <<  "The size of cfree_obj : " << cfolr.size() << std::endl;
		lcobj = DBL_MAX;
	}
	else{
		lcobj = fc_eval(cfolr[0]);
	}

	// moving right hand first, next left hand
	init();
	fcang = ini;
	controller->shape_update(goal);
	simple_narrow(begin_right);
	Node fcang_right = fcang;
	controller->robot_update(fcang_right);
	if(controller->RintersectR()){
		std::cout << "Robot intersect" << std::endl;
		rcobj = DBL_MAX;
	}
	std::vector<PointCloud> cforl = raster->extract(pc, fcang_right);
	if(cforl.size() != 1){
		std::cout <<  "The size of cfree_obj : " << cforl.size() << std::endl;
		rcobj = DBL_MAX;
	}
	else{
		rcobj = fc_eval(cforl[0]);
	}

	std::cout << "Left start: " << lcobj << "  ";
	std::cout << "Right start: " << rcobj << std::endl;
	if (lcobj > rcobj) {
		fcang = fcang_right;
	}
	else {
		fcang = fcang_left;
	}

}


Node FormClosure::get_fcangle()
{
	return fcang;
}


double FormClosure::fc_eval(PointCloud pc)
{
	double maxi = DBL_MIN;
	for(int i=0; i<pc.size(); ++i){
		double tmp = calc_dist(pc.get(i), goal);
		if(tmp > maxi)	maxi = tmp;
	}
	return maxi;
}







