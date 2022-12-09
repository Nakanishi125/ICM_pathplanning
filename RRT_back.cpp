#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <stdlib.h>

#include "RRT.h"
#include "Robot.h"
#include "Collision.h"
#include "CFreeObj.h"
#include "CFreeDFS.h"

using namespace std::chrono;
int origin = -1;


RRT::RRT(ProblemDefinition pd)
	:task(pd), tree(pd.initial, origin), garound()
{}


bool RRT::Initialize()
{
	CFreeICS ics;
	for(int i=0; i<6;++i)	std::cout << task.initial.get_element(i) << ", "; std::cout << std::endl;
	if (!robot_update(task.initial))	return false;
	if (!ics.extract())	return false;
	std::vector<PointCloud> init_CFree = ics.getter();
	print_ICSs(init_CFree);

	std::cout << "Select the cluster:";
	int index = -1;
	std::cin >> index;
	assert(0 <= index && index < (int)init_CFree.size());
	
	tree.replace(RRTNode(task.initial, init_CFree[index]));
	return true;
}


void RRT::planning()
{
	rand_init();

	if (!Initialize()) {
		std::cout << "Invalid initial value was given." << std::endl;
		return;
	}
	auto start = std::chrono::system_clock::now();

	while (1)
	{
		static int i = 0;
		++i;
		if (i > 200000)	exit(5963);
		// Random sampling and format
		Node Rand = generate_newnode();
		Node newnode = sampling(Rand);

		// Validation
		if (!robot_update(newnode))	continue;
		//if (!config_valid(newnode))	continue;
		if (!dfsconfig_valid(newnode))	continue;
		//if (!config_debug(newnode)) continue;

		// Post process
		int judgeflag = task.judge_once(tree.getRRTNode(tree.size()-1).getPC());
		if (judgeflag == 1)	add_garound();
		if (judgeflag == 2)	break;
	}

	auto end = std::chrono::system_clock::now();
	auto dur = end - start;
	auto sec = std::chrono::duration_cast<std::chrono::seconds>(dur).count();
	std::cout << "Elapsed time [s] :";	std::cout << sec << std::endl;
}




Node RRT::sampling(Node Rand)
{
	static int loop = 0;
	loop++;
	Node newnode;
	if (garound.size() < 10)	newnode = tree.format(Rand);
	else {
		if (loop % 10 == 3 || loop % 10 == 7)	newnode = tree.format(Rand);
		else                                    newnode = format_around(Rand);
	}
	std::cout << loop << ": ";
	for (int i = 0; i < 6; ++i)	std::cout <<  newnode.get_element(i) << ", ";	std::cout << std::endl;
	return newnode;
}


bool RRT::robot_update(Node newnode)	// Valid node -> true,  Invalid node -> false
{
	Robot* myRobo = Robot::get_instance();
	myRobo->update(newnode);

	if (myRobo->rr_intersect()) {
		tree.pop_back();
		return false;
	}
	if (Collision::RW_intersect()) {
		tree.pop_back();
		return false;
	}
	return true;
}


bool RRT::config_valid(Node newnode)	// Valid node -> true,  Invalid node -> false
{
	CFreeObj config(tree.getParentNode(-1).getPC());

	if (config.extract()) {
		RRTNode validnode(newnode, config.get_cfree_obj());
		tree.replace(validnode);
		return true;
	}
	else {
		tree.pop_back();
		return false;
	}
}


bool RRT::dfsconfig_valid(Node newnode)
{
	CFreeEdge cedge(tree.getParentNode(-1).getPC());
	if (cedge.extract()) {
		RRTNode validnode(newnode, cedge.get_cfree_obj());
		tree.replace(validnode);
		return true;
	}
	else {
		tree.pop_back();
		return false;
	}
}

bool RRT::config_debug(Node newnode)
{
	CFreeObj config(tree.getParentNode(-1).getPC());
	CFreeEdge cedge(tree.getParentNode(-1).getPC());

	bool c1, c2;
	c1 = config.extract();
	c2 = cedge.extract();
	std::cout << c1 << ", " << c2 << std::endl;

	PointCloud p1, p2;
	p1 = config.get_cfree_obj();
	p2 = cedge.get_cfree_obj();
	std::cout << "raster:" << p1.size() << std::endl;
	for (int i = 0; i < p1.size(); ++i) {
		if (!p2.exist(p1.get(i)))	std::cout << "(" << p1.get(i).x << ", " << p1.get(i).y << ", " << p1.get(i).th << ")" <<  std::endl;
	}
	std::cout << "dfs   :" << p2.size() << std::endl;
	for (int i = 0; i < p2.size(); ++i) {
		if (!p1.exist(p2.get(i)))	std::cout << "(" << p2.get(i).x << ", " << p2.get(i).y << ", " << p2.get(i).th << ")" << std::endl;
	}

	if (config.extract()) {
		RRTNode validnode(newnode, config.get_cfree_obj());
		tree.replace(validnode);
		return true;
	}
	else {
		tree.pop_back();
		return false;
	}
}


Node RRT::format_around(Node rand)
{
	double dist = DBL_MAX;
	int index = -1;
	for (int i = 0; i < (int)garound.size(); ++i) {
		double tmp = tree.getRRTNode(garound[i]).distance(rand);
		if (dist > tmp) {
			//index = tree.getParentIndex(garound[i]);
			index = garound[i];	// Change above line to this one (10/17)
			dist = tmp;
		}
	}

	Node fmt = rand.normalize(tree.getRRTNode(index).getNode());
	tree.push_back(fmt, index);
	return fmt;
}

void RRT::add_garound()
{
	garound.push_back(tree.size() - 1);
	std::cout << "around goal: " << garound.size() << std::endl;
}


void RRT::print_ICSs(std::vector<PointCloud> pcs)
{
	int num = 0;
	for (const auto& cls : pcs) {
		std::cout << num;	std::cout << ":" << cls.size() << std::endl;
		for (int i = 0; i < (int)cls.size(); ++i) {
			std::cout << "[";	std::cout << cls.get(i).x;	std::cout << ",";
			std::cout << cls.get(i).y;	std::cout << ",";	std::cout << cls.get(i).th;	std::cout << "],";
		//	if (i > 4)	break;
		}
		std::cout << std::endl << std::endl;
		++num;
	}
}


void RRT::pathprint_IO()
{
	NodeList path = tree.generate_path();
	path.reverse();
	path.printIO();
}


void RRT::pathprint_f(std::string fn)
{
	NodeList path = tree.generate_path();
	path.reverse();
	path.print_file(fn);
}


void rand_init()
{
	auto seed = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count() % 100000;
	//std::srand((unsigned int)seed);
	std::srand(91600);
	std::cout << "Seed value is " << seed << std::endl;
}


Node generate_newnode()
{
	std::vector<double> vecrand(6);
	for (int i = 0; i < 6; ++i) {
		vecrand[i] = std::rand() % 181 - 90;
	}
	return Node(vecrand);
}



RevRRT::RevRRT(Node ini)
	:tree(ini, origin)
{}


bool RevRRT::robot_update(Node newnode)
{
	Robot* myRobo = Robot::get_instance();
	myRobo->update(newnode);

	if (myRobo->rr_intersect())		return false;
	if (Collision::RW_intersect())	return false;

	return true;
}


bool RevRRT::config_valid(Node newnode)
{
	CFreeICS* ics = new CFreeICS();
	if (!ics->extract())	return false;

	std::vector<PointCloud> pcs = ics->getter();
	std::vector<PointCloud> del_list;
	int trunc = negligible_threshold();
	delete ics;

	for (PCSiterator it = pcs.begin(); it != pcs.end(); ) {
		RRTbool judge = check_overlap(*it, trunc);
		if (judge == RRTbool::Delete) {
			it = pcs.erase(it);
			continue;
		}
		else if (judge == RRTbool::False) {
			del_list.push_back(*it);
			it = pcs.erase(it);
			continue;
		}

		judge = check_hidden_overlap(*it);
		if (judge == RRTbool::False) {
			del_list.push_back(*it);
			it = pcs.erase(it);
			continue;
		}
		else if (judge == RRTbool::True) {
			++it;
		}
	}

	if ((int)pcs.size() == 0)	return false;
	else {
		RRTNode validnode(newnode, pcs, del_list);
		tree.replace(validnode);
		return true;
	}
}


Node RevRRT::sampling(Node Rand)
{
	static int loop = 0;
	loop++;
	Node newnode;
	//if (garound.size() < 10)	newnode = tree.format(Rand);
	//else {
	//	if (loop % 10 == 3 || loop % 10 == 7)	newnode = tree.format(Rand);
	//	else                                    newnode = format_around(Rand);
	//}
	newnode = tree.format(Rand);
	std::cout << loop << ": ";
	for (int i = 0; i < 6; ++i)	std::cout <<  newnode.get_element(i) << ", ";	std::cout << std::endl;

	return newnode;
}


void RevRRT::planning()
{
	Initialize();
	rand_init();
	GoalJudge goal;

	while (1)
	{
		static int all = 0;
		static int hit = 0;
		++all;
		// Random sampling and formatting
		Node Rand = generate_newnode();
		Node newnode = sampling(Rand);

		// Valid or not
		if (!robot_update(newnode)) {
			tree.pop_back();
			continue;
		}
		if (!config_valid(newnode)) {
			tree.pop_back();
			continue;
		}

		++hit;
		if (hit % 10 == 0)	std::cout << "Hit rate: " << 100 * (double)hit / all << std::endl;
		bool flag = goal.judge_upper(tree.getRRTNode(-1).get_cfree_obj());
		if (flag == true)	break;

	}


}


bool RevRRT::Initialize()
{
	CFreeICS ics;

	if (!robot_update(tree.getRRTNode(0).getNode()))	return false;
	if (!ics.extract())	return false;
	std::vector<PointCloud> init_CFree = ics.getter();
	
	print_ICSs(init_CFree);
	std::cout << "Select the cluster:";
	int index = -1;
	std::cin >> index;
	assert(0 <= index && index < (int)init_CFree.size());
	PointCloud selected = init_CFree[index];
	init_CFree.erase(init_CFree.begin() + index);

	tree.replace(RRTNode(tree.getRRTNode(0).getNode(), { selected }, init_CFree));
	return true;
}


void RevRRT::print_ICSs(std::vector<PointCloud> pcs)
{
	int num = 0;
	for (const auto& cls : pcs) {
		std::cout << num;	std::cout << ":" << std::endl;
		for (int i = 0; i < (int)cls.size(); ++i) {
			std::cout << "[";	std::cout << cls.get(i).x;	std::cout << ",";
			std::cout << cls.get(i).y;	std::cout << ",";	std::cout << cls.get(i).th;	std::cout << "],";
		}
		std::cout << std::endl << std::endl;
		++num;
	}
}


int RevRRT::negligible_threshold()
{
	std::vector<PointCloud> pcs = tree.getParentNode(-1).get_cfree_obj();
	const int del_threshold = 20;
	int point_num = 0;
	for (int i = 0; i < (int)pcs.size(); ++i) {
		point_num += pcs[i].size();
	}

	assert((int)pcs.size() != 0);
	int ave_size = point_num / (int)pcs.size();		//Ø‚èŽÌ‚Ä
	return ave_size / del_threshold;
}


RRTbool RevRRT::check_overlap(PointCloud pc, int trunc)
{
	int laps = 0;
	const int threshold = 500;
	std::vector<PointCloud> parent_obj = tree.getParentNode(-1).get_cfree_obj();

	if (pc.size() <= trunc) {
		return RRTbool::Delete;
	}

	for (int i = 0; i < (int)parent_obj.size(); ++i) {
		if (pc.overlap(parent_obj[i])) {
			float rate = ((int)parent_obj[i].size() > pc.size())
				? (float)(parent_obj[i].size()) / pc.size()
				: pc.size() / (float)(parent_obj[i].size());
			if (rate > threshold) {
				continue;
			}
			++laps;
		}
	}

	if (laps != 1) {
		return RRTbool::False;
	}
	else            return RRTbool::True;
}

RRTbool RevRRT::check_hidden_overlap(PointCloud pc)
{
	std::vector<PointCloud> parent_del = tree.getParentNode(-1).get_cfree_del();

	for (int i = 0; i < (int)parent_del.size(); ++i) {
		if (pc.overlap(parent_del[i])) {
			return RRTbool::False;
		}
	}
	return RRTbool::True;
}


//bool RevRRT::config_valid(Node newnode)
//{
//	int del_threshold = 50;
//	int threshold = 10;
//	CFreeICS* ics = new CFreeICS();
//	if (!ics->extract())	return false;
//	std::vector<PointCloud> pcs = ics->getter();
//	std::vector<PointCloud> del_list;
//
//	// pcs‚Ætree.ParentNode(-1).getPC()‚Å‘Ã“–‚È‚à‚Ì‚¾‚¯Žc‚·
//	std::vector<PointCloud> parent_obj = tree.getParentNode(-1).get_cfree_obj();
//	std::vector<PointCloud> parent_del = tree.getParentNode(-1).get_cfree_del();
//
//	int point_num = 0;
//	for (int i = 0; i < (int)parent_obj.size(); ++i) {
//		point_num += parent_obj[i].size();
//	}
//	double ave_size = (double)point_num / (int)parent_obj.size();
//	int trunc = ave_size / del_threshold;
//
//
//	for (std::vector<PointCloud>::iterator it = pcs.begin(); it != pcs.end();) {
//		if ((*it).size() <= trunc) {
//			it = pcs.erase(it);
//			continue;
//		}
//
//		int laps = 0;
//		for (int i = 0; i < (int)parent_obj.size(); ++i) {
//			if ((*it).overlap(parent_obj[i])) {
//				double rate;
//				if ((int)parent_obj.size() > (*it).size()) {
//					rate = (int)parent_obj.size() / (double)(*it).size();
//				}
//				else {
//					rate = (double)(*it).size() / (int)parent_obj.size();
//				}
//
//				if (rate > threshold)	continue;
//				laps++;
//			}
//		}
//		if (laps == 0) {
//			del_list.push_back(*it);
//			it = pcs.erase(it);
//			continue;
//		}
//
//		for (int i = 0; i < (int)parent_del.size(); ++i) {
//			if ((*it).overlap(parent_del[i])) {
//				laps++;
//			}
//		}
//
//		if (laps != 1) {
//			del_list.push_back(*it);
//			it = pcs.erase(it);
//		}
//		else {
//			++it;
//		}
//	}
//
//	if ((int)pcs.size() == 0) {
//		tree.pop_back();
//		return false;
//	}
//	else {
//		RRTNode validnode(newnode, pcs, del_list);
//		tree.replace(validnode);
//		return true;
//	}
//}


RRTConnect::RRTConnect(Node ini, Node fin)
	:stree(ini, origin), gtree(fin, origin)
{}

bool RRTConnect::initialize()
{
	CFreeICS ics_start, ics_goal;

	if (!robot_update(stree.getRRTNode(0).getNode()))	return false;
	if (!ics_start.extract())	return false;
	std::vector<PointCloud> init_CFree = ics_start.getter();
	print_ICSs(init_CFree);
	std::cout << "Select the cluster (start configuration):";
	int index = -1;
	std::cin >> index;
	assert(0 <= index && index < (int)init_CFree.size());

	stree.replace(RRTNode(stree.getRRTNode(0).getNode(), init_CFree[index]));

	if (!robot_update(gtree.getRRTNode(0).getNode()))	return false;
	if (!ics_goal.extract())	return false;
	std::vector<PointCloud> fin_CFree = ics_goal.getter();
	print_ICSs(fin_CFree);
	std::cout << "Select the cluster (goal configuration):";
	int index2 = -1;
	std::cin >> index2;
	assert(0 <= index2 && index2 < (int)fin_CFree.size());
	PointCloud selected = fin_CFree[index2];
	fin_CFree.erase(fin_CFree.begin() + index2);

	gtree.replace(RRTNode(gtree.getRRTNode(0).getNode(), { selected }, fin_CFree));

	return true;
}

NodeList RRTConnect::planning()
{
	initialize();
	rand_init();
	GoalJudge goal;

	while (1)
	{
		// Random sampling and formatting
		Node Rand = generate_newnode();
		RRTNode newnode;
		bool judge = false;

		if (stree.size() < gtree.size()) {
			Node add = stree.format(Rand);
			RRTNode nearest = gtree.get_nearest_node(add);
			double tmpdist = add.distance(nearest.getNode());
			std::cout << "now distance: " << tmpdist << std::endl;
			for (int i = 0; i < 6; ++i)	std::cout << std::fixed << std::setprecision(1) << add.get_element(i) << ", "; std::cout << std::endl;

			if (!robot_update(add)) {
				stree.pop_back();
				continue;
			}
			if (!sconfig_valid(add)) {
				stree.pop_back();
				continue;
			}

			std::cout << "start conf size: " << stree.size() << " goal conf size: " << gtree.size() << std::endl;
			RRTNode addnode = stree.getRRTNode(-1);
			judge = goal.judge_sside(addnode.getPC());
			if (judge == true) {
				std::cout << "Start conf reached to goal" << std::endl;
				return stree.generate_path();
			}

			judge = goal.judge_connect(addnode, nearest);
			if (judge == true) {
				std::cout << "Connect goal!" << std::endl;
				return path_concat();
			}

			newnode = nearest;

			while (1)
			{
				newnode = newnode.getNode().unitmove(addnode.getNode());
				gtree.push_back(newnode, gtree.get_now_index());
				for (int i = 0; i < 6; ++i)	std::cout << std::fixed << std::setprecision(1) << newnode.getNode().get_element(i) << ", "; std::cout << std::endl;

				if (!robot_update(newnode.getNode())) {
					gtree.pop_back();
					break;
				}
				if (!gconfig_valid(newnode.getNode())) {
					gtree.pop_back();
					break;
				}

				std::cout << "start conf size: " << stree.size() << " goal conf size: " << gtree.size() << std::endl;
				newnode = gtree.getRRTNode(-1);
				judge = goal.judge_gside(newnode.get_cfree_obj());
				if (judge == true) {
					std::cout << "Goal conf reached to goal" << std::endl;
					return gtree.generate_path();
				}

				judge = goal.judge_connect(addnode, newnode);
				if (judge == true) {
					std::cout << "Connect goal!" << std::endl;
					return path_concat();
				}

				double threshold = addnode.distance(newnode);
				if (threshold < 1.0) {
					std::cout << "break;";
					break;
				}
			}
			
		}

		else {
			Node add = gtree.format(Rand);
			RRTNode nearest = stree.get_nearest_node(add);
			double tmpdist = add.distance(nearest.getNode());
			std::cout << "now distance: " << tmpdist << std::endl;

			for (int i = 0; i < 6; ++i)	std::cout << std::fixed << std::setprecision(1) << add.get_element(i) << ", "; std::cout << std::endl;
			
			if (!robot_update(add)) {
				gtree.pop_back();
				continue;
			}
			if (!gconfig_valid(add)) {
				gtree.pop_back();
				continue;
			}

			std::cout << "start conf size: " << stree.size() << " goal conf size: " << gtree.size() << std::endl;
			RRTNode addnode = gtree.getRRTNode(-1);
			judge = goal.judge_gside(addnode.get_cfree_obj());
			if (judge == true) {
				std::cout << "Goal conf reached to goal" << std::endl;
				NodeList path = gtree.generate_path();
				path.reverse();
				return path;
			}

			judge = goal.judge_connect(nearest, addnode);
			if (judge == true) {
				std::cout << "Connect goal!" << std::endl;
				return path_concat();
			}

			newnode = nearest;

			while(1)
			{
				newnode = newnode.getNode().unitmove(addnode.getNode());
				stree.push_back(newnode, stree.get_now_index());
				for (int i = 0; i < 6; ++i)	std::cout << std::fixed << std::setprecision(1) << newnode.getNode().get_element(i) << ", "; std::cout << std::endl;

				if (!robot_update(newnode.getNode())) {
					stree.pop_back();
					break;
				}
				if (!sconfig_valid(newnode.getNode())) {
					stree.pop_back();
					break;
				}

				std::cout << "start conf size: " << stree.size() << " goal conf size: " << gtree.size() << std::endl;
				newnode = stree.getRRTNode(-1);
				judge = goal.judge_sside(newnode.getPC());
				if (judge == true) {
					std::cout << "Start conf reached to goal" << std::endl;
					return stree.generate_path();
				}

				judge = goal.judge_connect(newnode, addnode);
				if (judge == true) {
					std::cout << "Connect goal!" << std::endl;
					return path_concat();
				}

				double threshold = addnode.distance(newnode);
				if (threshold < 1.0) {
					std::cout << "break;";
					break;
				}
			}

		}
	}
}

bool RRTConnect::robot_update(Node newnode)
{
	Robot* myRobo = Robot::get_instance();
	myRobo->update(newnode);

	if (myRobo->rr_intersect())	return false;
	if (Collision::RW_intersect())	return false;

	return true;
}

void RRTConnect::print_ICSs(std::vector<PointCloud> pcs)
{
	int num = 0;
	for (const auto& cls : pcs) {
		std::cout << num;	std::cout << ":" << std::endl;
		for (int i = 0; i < (int)cls.size(); ++i) {
			std::cout << "[";	std::cout << cls.get(i).x;	std::cout << ",";
			std::cout << cls.get(i).y;	std::cout << ",";	std::cout << cls.get(i).th;	std::cout << "],";
			//	if (i > 4)	break;
		}
		std::cout << std::endl << std::endl;
		++num;
	}
}

Node RRTConnect::sampling(Node Rand)
{
	Node newnode;
	if (stree.size() < gtree.size())	newnode = stree.format(Rand);
	else                                newnode = gtree.format(Rand);

	for (int i = 0; i < 6; ++i)	std::cout << std::fixed << std::setprecision(1) << newnode.get_element(i) << ", ";	std::cout << std::endl;

	return newnode;
}

bool RRTConnect::gconfig_valid(Node newnode)
{
	CFreeICS* ics = new CFreeICS();
	if (!ics->extract())	return false;

	std::vector<PointCloud> pcs = ics->getter();
	std::vector<PointCloud> del_list;
	int trunc = negligible_threshold();
	delete ics;

	for (PCSiterator it = pcs.begin(); it != pcs.end(); ) {
		RRTbool judge = check_overlap(*it, trunc);
		if (judge == RRTbool::Delete) {
			it = pcs.erase(it);
			continue;
		}
		else if (judge == RRTbool::False) {
			del_list.push_back(*it);
			it = pcs.erase(it);
			continue;
		}

		judge = check_hidden_overlap(*it);
		if (judge == RRTbool::False) {
			del_list.push_back(*it);
			it = pcs.erase(it);
			continue;
		}
		else if (judge == RRTbool::True) {
			++it;
		}
	}

	if ((int)pcs.size() == 0) return false;
	else {
		RRTNode validnode(newnode, pcs, del_list);
		gtree.replace(validnode);
		return true;
	}
}

bool RRTConnect::sconfig_valid(Node newnode)	// Valid node -> true,  Invalid node -> false
{
	CFreeObj config(stree.getParentNode(-1).getPC());

	if (config.extract()) {
		RRTNode validnode(newnode, config.get_cfree_obj());
		stree.replace(validnode);
		return true;
	}
	return false;
}

RRTbool RRTConnect::check_overlap(PointCloud pc, int trunc)
{
	int laps = 0;
	const int threshold = 500;
	std::vector<PointCloud> parent_obj = gtree.getParentNode(-1).get_cfree_obj();

	if (pc.size() <= trunc) {
		return RRTbool::Delete;
	}

	for (int i = 0; i < (int)parent_obj.size(); ++i) {
		if (pc.overlap(parent_obj[i])) {
			float rate = ((int)parent_obj[i].size() > pc.size())
				? (float)(parent_obj[i].size()) / pc.size()
				: pc.size() / (float)(parent_obj[i].size());
			if (rate > threshold) {
				continue;
			}
			++laps;
		}
	}

	if (laps != 1) {
		return RRTbool::False;
	}
	else            return RRTbool::True;
}

RRTbool RRTConnect::check_hidden_overlap(PointCloud pc)
{
	std::vector<PointCloud> parent_del = gtree.getParentNode(-1).get_cfree_del();

	for (int i = 0; i < (int)parent_del.size(); ++i) {
		if (pc.overlap(parent_del[i])) {
			return RRTbool::False;
		}
	}
	return RRTbool::True;
}

int RRTConnect::negligible_threshold()
{
	std::vector<PointCloud> pcs = gtree.getParentNode(-1).get_cfree_obj();
	const int del_threshold = 20;
	int point_num = 0;
	for (int i = 0; i < (int)pcs.size(); ++i) {
		point_num += pcs[i].size();
	}

	assert((int)pcs.size() != 0);
	int ave_size = point_num / (int)pcs.size();		//Ø‚èŽÌ‚Ä
	return ave_size / del_threshold;
}
