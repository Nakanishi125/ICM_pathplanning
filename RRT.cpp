#include<iostream>
#include<chrono>
#include<climits>

#include "RRT.h"
#include "CFreeICS.h"
#include "Robot.h"
#include "CFree.h"


int origin = -1;
using namespace std::chrono;

void RRT::set_strategy(CFO* cfo)
{
	strategy = cfo;
}


bool RRT::initialize(Node ini)
{
	tree.push_back(ini, origin);
	CFreeICS ics(ini);
	for(int i=0; i<6;++i)	std::cout << ini.get_element(i) << ", "; std::cout << std::endl;
	if (!robot_update(ini))	return false;
	std::vector<PointCloud> init_CFree = ics.extract();
	if(init_CFree.size() == 0)	return false;
	print_ICSs(init_CFree);

	std::cout << "Select the cluster:";
	int index = -1;
	std::cin >> index;
	assert(0 <= index && index < (int)init_CFree.size());
	
	tree.replace(RRTNode(ini, init_CFree[index]));
	return true;
}


bool RRT::dfsconfig_valid(Node newnode)
{
	std::vector<PointCloud> cfo = strategy->extract(tree.back_parentRRTNode().pc(), newnode);

	if((int)cfo.size() == 1){
		RRTNode validnode(newnode, cfo[0]);
		tree.replace(validnode);
		return true;
	}
	else {
		return false;
	}

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
	for (int i = 0; i < 6; ++i)	std::cout << newnode.get_element(i) << ", ";	std::cout << std::endl;
	return newnode;
}


void RRT::add_garound()
{
	garound.push_back(tree.size() - 1);
	std::cout << "around goal: " << garound.size() << std::endl;
}


Node RRT::format_around(Node rand)
{
	double dist = DBL_MAX;
	int index = -1;
	for (int i = 0; i < (int)garound.size(); ++i) {
		double tmp = tree.get_RRTNode(garound[i]).distance(rand);
		if (dist > tmp) {
			//index = tree.getParentIndex(garound[i]);
			index = garound[i];	// Change above line to this one (10/17)
			dist = tmp;
		}
	}

	Node fmt = rand.normalize(tree.get_RRTNode(index).node);
	tree.push_back(fmt, index);
	return fmt;
}


GoalJudge RRT::goal_judge(State3D goal)
{
	PointCloud pc = tree.back_RRTNode().pc();
    const double around_rate = 1.5;
    double max_dist = DBL_MIN;
    int xmin = INT_MAX, xmax = INT_MIN;

    for (int i = 0; i < pc.size(); ++i) {
        double dist = calc_dist(pc.get(i), goal);
        if (dist > threshold * around_rate)  return GoalJudge::NotGoal;

        int xtmp = pc.get(i).x;
        if (xtmp < xmin) xmin = xtmp;
        if (xtmp > xmax) xmax = xtmp;

        if (max_dist < dist)   max_dist = dist;
    }
    
    int delta_x = (xmax - xmin) / 2;
    max_dist = std::sqrt(max_dist * max_dist + delta_x * delta_x);
    if (max_dist > threshold * around_rate) return GoalJudge::NotGoal;

    std::cout << "max distance:" << max_dist << std::endl;

    if (!contain_yth(pc, goal)) {
        std::cout << "epsilon is satisfied but doesn't contain goal state." << std::endl;
        return GoalJudge::NotGoal;
    }

    if (max_dist > threshold)    return GoalJudge::MiddleGoal;
    else                         return GoalJudge::Goal;
}


RRT::RRT()
	:tree(), garound(), strategy(new DfsCFO()), 
  	threshold(read_threshold())
{
	set_strategy(new RasterCFO());
}


NodeList RRT::plan(Node ini, Node fin, State3D goal)
{
	rand_init();

	if (!initialize(ini)) {
		std::cout << "Invalid initial value was given." << std::endl;
		return NodeList();
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
		if (!robot_update(newnode)){
			tree.pop_back();
			continue;
		}
		if (!dfsconfig_valid(newnode)){
			tree.pop_back();
			continue;
		}

		// Post process
		GoalJudge flag = goal_judge(goal);
		if (flag == GoalJudge::MiddleGoal)	add_garound();
		if (flag == GoalJudge::Goal)	break;
	}

	auto end = std::chrono::system_clock::now();
	auto dur = end - start;
	auto sec = std::chrono::duration_cast<std::chrono::seconds>(dur).count();
	std::cout << "Elapsed time [s] :";	std::cout << sec << std::endl;

	return tree.generate_path();
}


// ===============================================================================


bool RevRRT::initialize(Node fin)
{
	tree.push_back(fin, origin);
	CFreeICS ics(fin);

	for(int i=0; i<6;++i)	std::cout << fin.get_element(i) << ", "; std::cout << std::endl;
	if (!robot_update(fin))	return false;

	std::vector<PointCloud> fin_CFree = ics.extract();
	if(fin_CFree.size() == 0)	return false;
	print_ICSs(fin_CFree);

	std::cout << "Select the cluster:";
	int index = -1;
	std::cin >> index;
	assert(0 <= index && index < (int)fin_CFree.size());
	PointCloud selected = fin_CFree[index];
	fin_CFree.erase(fin_CFree.begin() + index);
	tree.replace(RRTNode(fin, {selected}, fin_CFree));

	return true;
}


//bool RevRRT::dfsconfig_valid(Node newnode)
//{
//	Controller* controller = Controller::get_instance();
//
//	std::vector<PointCloud> prev_cfree_obj = tree.back_parentRRTNode().get_cfree_obj();
//	std::vector<PointCloud> prev_cfree_del = tree.back_parentRRTNode().get_cfree_del();
//
//	std::vector<PointCloud> cfree_obj;
//	std::vector<PointCloud> del_list;
//
//	for(const auto& eo: prev_cfree_obj){
//		std::vector<PointCloud> cfree_obj_tmp = strategy->extract(eo, newnode);
//		for(auto it = cfree_obj_tmp.begin(); it != cfree_obj_tmp.end(); ){
//			int flag = 0;
//			for(const auto& ed: prev_cfree_del){
//				if((*it).overlap(ed)){
//					flag = 1;	break;
//				}
//			}
//			if(flag == 1){
//				del_list.push_back(*it);
//				it = cfree_obj_tmp.erase(it);
//			}
//			else{
//				++it;
//			}
//		}
//
//		for(const auto& e : cfree_obj_tmp){
//			if(duplicate_check(e, cfree_obj))	continue;
//			cfree_obj.push_back(e);
//		}
//	}
//
//	
//	Node parent = tree.back_parentRRTNode().node;
//	controller->robot_update(parent);
//	for(auto it = cfree_obj.begin(); it != cfree_obj.end(); ){
//		std::vector<PointCloud> prev_real_cfree = strategy->extract(*it, parent);
//		if(prev_real_cfree.size() != 1){
//			del_list.push_back(*it);
//			it = cfree_obj.erase(it);
//		}
//		else{
//			++it;
//		}
//	}
//	controller->robot_update(newnode);
//
//	if((int)cfree_obj.size() == 0)	return false;
//	else{
//		RRTNode validnode(newnode, cfree_obj, del_list);
//		tree.replace(validnode);
//		return true;
//	}
//}


bool RevRRT::dfsconfig_valid(Node newnode)
{
	Controller* controller = Controller::get_instance();

	std::vector<PointCloud> prev_cfree_obj = tree.back_parentRRTNode().get_cfree_obj();
	std::vector<PointCloud> prev_cfree_del = tree.back_parentRRTNode().get_cfree_del();

	std::vector<PointCloud> cfree_obj;
	std::vector<PointCloud> del_list;

	for(const auto& eo: prev_cfree_obj){
		std::vector<PointCloud> cfree_obj_tmp = strategy->extract(eo, newnode);

		for(const auto& e : cfree_obj_tmp){
			if(duplicate_check(e, cfree_obj))	continue;
			cfree_obj.push_back(e);
		}
	}

	
	Node parent = tree.back_parentRRTNode().node;
	controller->robot_update(parent);
	for(auto it = cfree_obj.begin(); it != cfree_obj.end(); ){
		std::vector<PointCloud> prev_real_cfree = strategy->extract(*it, parent);
		if(prev_real_cfree.size() != 1){
			del_list.push_back(*it);
			it = cfree_obj.erase(it);
		}
		else{
			++it;
		}
	}
	controller->robot_update(newnode);

	if((int)cfree_obj.size() == 0)	return false;
	else{
		RRTNode validnode(newnode, cfree_obj, del_list);
		tree.replace(validnode);
		return true;
	}
}



Node RevRRT::sampling(Node Rand)
{
	static int loop = 0;
	loop++;
	Node newnode = tree.format(Rand);
	std::cout << loop << ": ";
	for(int i=0; i<6; ++i)	std::cout << newnode.node[i] << ", ";	std::cout << std::endl;

	return newnode;
}


GoalJudge RevRRT::goal_judge(std::vector<PointCloud> pcs)
{
    int minx = INT_MAX, miny = INT_MAX, mint = INT_MAX;
    int maxx = INT_MIN, maxy = INT_MIN, maxt = INT_MIN;
    bool flag = false;

//    for (int i = 0; i < (int)pcs.size(); ++i) {
//        for (int j = 0; j < (int)pcs[i].size(); ++j) {
//            /*if (minx > pcs[i].get(j).x)  minx = pcs[i].get(j).x;
//            if (maxx < pcs[i].get(j).x)  maxx = pcs[i].get(j).x;*/
//            if (miny > pcs[i].get(j).y)  miny = pcs[i].get(j).y;
//            if (maxy < pcs[i].get(j).y)  maxy = pcs[i].get(j).y;
//            if (mint > pcs[i].get(j).th)  mint = pcs[i].get(j).th;
//            if (maxt < pcs[i].get(j).th)  maxt = pcs[i].get(j).th;
//            if (pcs[i].get(j).th == 0)   flag = true;
//        }
//        int widy = (maxy - miny) / 10;
//        int widt = (maxt - mint) / 5;
//        
//        if (flag) {
//            int pmint = INT_MAX, pmaxt = INT_MIN;
//            for (int j = 0; j < (int)pcs[i].size(); ++j) {
//                if (pcs[i].get(j).th < 180) {
//                    if (pcs[i].get(j).th > pmaxt)    pmaxt = pcs[i].get(j).th;
//                }
//                if (pcs[i].get(j).th >= 180) {
//                    if (pcs[i].get(j).th < pmint)    pmint = pcs[i].get(j).th;
//                }
//            }
//            widt = (pmaxt + (360 - pmint)) / 5;
//        }
//
//        std::cout << "y width: " << widy << std::endl;
//        std::cout << "theta width: " << widt << std::endl;
//        if (widy < 8)      return GoalJudge::NotGoal;
//        if (widt < 25)      return GoalJudge::NotGoal;
//    }

	for(int i=0; i<(int)pcs.size(); ++i){
		if(pcs[i].size() > 1000)	return GoalJudge::Goal;
	}
    return GoalJudge::NotGoal;
}


RevRRT::RevRRT()
	:tree(), strategy(new DfsCFO())
{}


NodeList RevRRT::plan(Node ini, Node fin, State3D goal)
{
	rand_init();

	if(!initialize(fin)){
		std::cout << "Invalid initial value was given." << std::endl;
		return NodeList();
	}

	while(1)
	{
		static int i = 0;	++i;
		if(i>200000)	exit(5963);
		
		// Random sampling and format
		Node Rand = generate_newnode();
		Node newnode = sampling(Rand);

		// Validation
		if(!robot_update(newnode)){
			tree.pop_back();
			continue;
		}
		if(!dfsconfig_valid(newnode)){
			tree.pop_back();
			continue;
		}

		GoalJudge flag = goal_judge(tree.back_RRTNode().get_cfree_obj());
		if(flag == GoalJudge::Goal)	break;
	}
	
	NodeList path = tree.generate_path();
	path.reverse();
	return path;
}


//////////////////////////////////////////////////////////////////////////

bool RRTConnect::initialize(Node ini, Node fin)
{
	CFreeICS ini_ics(ini), fin_ics(fin);
	s_tree.push_back(ini, origin);
	g_tree.push_back(fin, origin);
	int index1 = -1, index2 = -1;

	// Setting initial configuration
	if(!robot_update(ini))	return false;
	std::vector<PointCloud> ini_CFree = ini_ics.extract();
	if(ini_CFree.size() == 0)	return false;
	print_ICSs(ini_CFree);

	std::cout << "Select a cluster (start configuration):";
	std::cin >> index1;
	assert(0 <= index1 && index1 < (int)ini_CFree.size());
	s_tree.replace(RRTNode(ini, ini_CFree[index1]));

	// Setting goal configuration	
	if(!robot_update(fin))	return false;
	std::vector<PointCloud> fin_CFree = fin_ics.extract();
	if(fin_CFree.size() == 0)	return false;
	print_ICSs(fin_CFree);

	std::cout << "Select a cluster (goal configuration):";
	std::cin >> index2;
	assert(0 <= index2 && index2 < (int)fin_CFree.size());
	PointCloud selected = fin_CFree[index2];
	fin_CFree.erase(fin_CFree.begin() + index2);
	g_tree.replace(RRTNode(fin, {selected}, fin_CFree));

	return true;
}


bool RRTConnect::sconf_update()
{
	Node newnode = s_tree.back_RRTNode().node;
	if(!robot_update(newnode)){
		s_tree.pop_back();
		return false;
	}
	if(!caging_validation_sconf(newnode)){
		s_tree.pop_back();
		return false;
	}

	return true;
}


bool RRTConnect::gconf_update()
{
	Node newnode = g_tree.back_RRTNode().node;
	if(!robot_update(newnode)){
		g_tree.pop_back();
		return false;
	}
	if(!caging_validation_gconf(newnode)){
		g_tree.pop_back();
		return false;
	}

	return true;
}


GoalJudge RRTConnect::sconf_goaljudge(State3D goal, RRTNode bef, RRTNode aft)
{
	if(goal_sconf(goal) == GoalJudge::SGoal){
		std::cout << "Start conf reached to the goal" << std::endl;
		return GoalJudge::SGoal;
	}
	if(goal_connect(bef, aft) == GoalJudge::Connect){
		std::cout << "Connect goal!" << std::endl;
		return GoalJudge::Connect;
	}

	return GoalJudge::NotGoal;
}


GoalJudge RRTConnect::gconf_goaljudge(std::vector<PointCloud> cfo, RRTNode bef, RRTNode aft)
{
	if(goal_gconf(cfo) == GoalJudge::GGoal){
		std::cout << "Goal conf reached to desire start condition." << std::endl;
		return GoalJudge::GGoal;
	}
	if(goal_connect(bef, aft) == GoalJudge::Connect){
		std::cout << "Connect goal." << std::endl;
		return GoalJudge::Connect;
	}
	
	return GoalJudge::NotGoal;
}


bool RRTConnect::caging_validation_sconf(Node node)
{
	std::vector<PointCloud> cfo = strategy->extract(s_tree.back_parentRRTNode().pc(), node);

	if((int)cfo.size() == 1){
		RRTNode validnode(node, cfo[0]);
		s_tree.replace(validnode);
		return true;
	}
	else{
		return false;
	}
}


bool RRTConnect::caging_validation_gconf(Node node)
{
	Controller* controller = Controller::get_instance();

	std::vector<PointCloud> prev_cfree_obj = g_tree.back_parentRRTNode().get_cfree_obj();
	std::vector<PointCloud> prev_cfree_del = g_tree.back_parentRRTNode().get_cfree_del();

	std::vector<PointCloud> cfree_obj;
	std::vector<PointCloud> del_list;

	for(const auto& eo: prev_cfree_obj){
		std::vector<PointCloud> cfree_obj_tmp = strategy->extract(eo, node);
		for(auto it = cfree_obj_tmp.begin(); it != cfree_obj_tmp.end(); ){
			int flag = 0;
			for(const auto& ed: prev_cfree_del){
				if((*it).overlap(ed)){
					flag = 1;	break;
				}
			}
			if(flag == 1){
				del_list.push_back(*it);
				it = cfree_obj_tmp.erase(it);
			}
			else{
				++it;
			}
		}

		for(const auto& e : cfree_obj_tmp){
			if(duplicate_check(e, cfree_obj))	continue;
			cfree_obj.push_back(e);
		}
	}

	
	Node parent = g_tree.back_parentRRTNode().node;
	controller->robot_update(parent);
	for(auto it = cfree_obj.begin(); it != cfree_obj.end(); ){
		std::vector<PointCloud> prev_real_cfree = strategy->extract(*it, parent);
		if(prev_real_cfree.size() != 1){
			del_list.push_back(*it);
			it = cfree_obj.erase(it);
		}
		else{
			++it;
		}
	}
	controller->robot_update(node);

	if((int)cfree_obj.size() == 0)	return false;
	else{
		RRTNode validnode(node, cfree_obj, del_list);
		g_tree.replace(validnode);
		return true;
	}
}

GoalJudge RRTConnect::goal_sconf(State3D goal)
{
	PointCloud pc = s_tree.back_RRTNode().pc();
    double max_dist = DBL_MIN;
    int xmin = INT_MAX, xmax = INT_MIN;

    for (int i = 0; i < pc.size(); ++i) {
        double dist = calc_dist(pc.get(i), goal);
        if (dist > s_threshold)  return GoalJudge::NotGoal;

        int xtmp = pc.get(i).x;
        if (xtmp < xmin) xmin = xtmp;
        if (xtmp > xmax) xmax = xtmp;

        if (max_dist < dist)   max_dist = dist;
    }
    
    int delta_x = (xmax - xmin) / 2;
    max_dist = std::sqrt(max_dist * max_dist + delta_x * delta_x);
    if (max_dist > s_threshold) return GoalJudge::NotGoal;

    std::cout << "max distance:" << max_dist << std::endl;

    if (!contain_yth(pc, goal)) {
        std::cout << "epsilon is satisfied but doesn't contain goal state." << std::endl;
        return GoalJudge::NotGoal;
    }

	return GoalJudge::SGoal;
}

GoalJudge RRTConnect::goal_connect(RRTNode bef, RRTNode aft)
{
	double dist = bef.distance(aft);
	if(dist > 1.0)	return GoalJudge::NotGoal;

	std::vector<PointCloud> bef_cfree = bef.get_cfree_obj();
	std::vector<PointCloud> aft_cfree = aft.get_cfree_obj();
	std::vector<PointCloud> aft_cdel  = aft.get_cfree_del();

	for(const auto& bfree : bef_cfree){
		for(const auto& adel : aft_cdel){
			bool tof = bfree.overlap(adel);
			if(tof)	return GoalJudge::NotGoal;
		}
	}

	for(const auto& bfree: bef_cfree){
		for(const auto& afree: aft_cfree){
			bool tof = bfree.overlap(afree);
			if(tof)	return GoalJudge::Connect;
		}
	}

	return GoalJudge::NotGoal;
}


GoalJudge RRTConnect::goal_gconf(std::vector<PointCloud> cfo)
{
//    int minx = INT_MAX, miny = INT_MAX, mint = INT_MAX;
//    int maxx = INT_MIN, maxy = INT_MIN, maxt = INT_MIN;
//    bool flag = false;
//
//    for (int i = 0; i < (int)cfo.size(); ++i) {
//        for (int j = 0; j < (int)cfo[i].size(); ++j) {
//            /*if (minx > cfo[i].get(j).x)  minx = cfo[i].get(j).x;
//            if (maxx < cfo[i].get(j).x)  maxx = cfo[i].get(j).x;*/
//            if (miny > cfo[i].get(j).y)  miny = cfo[i].get(j).y;
//            if (maxy < cfo[i].get(j).y)  maxy = cfo[i].get(j).y;
//            if (mint > cfo[i].get(j).th)  mint = cfo[i].get(j).th;
//            if (maxt < cfo[i].get(j).th)  maxt = cfo[i].get(j).th;
//            if (cfo[i].get(j).th == 0)   flag = true;
//        }
//        int widy = (maxy - miny) / 10;
//        int widt = (maxt - mint) / 5;
//        
//        if (flag) {
//            int pmint = INT_MAX, pmaxt = INT_MIN;
//            for (int j = 0; j < (int)cfo[i].size(); ++j) {
//                if (cfo[i].get(j).th < 180) {
//                    if (cfo[i].get(j).th > pmaxt)    pmaxt = cfo[i].get(j).th;
//                }
//                if (cfo[i].get(j).th >= 180) {
//                    if (cfo[i].get(j).th < pmint)    pmint = cfo[i].get(j).th;
//                }
//            }
//            widt = (pmaxt + (360 - pmint)) / 5;
//        }
//
////        std::cout << "y width: " << widy << std::endl;
////        std::cout << "theta width: " << widt << std::endl;
//        if (widy < 8)      return GoalJudge::NotGoal;
//        if (widt < 18)      return GoalJudge::NotGoal;
//    }
//
//	  return GoalJudge::GGoal;
	
	for(int i=0; i<(int)cfo.size(); ++i){
		if(cfo[i].size() > 1500)	return GoalJudge::GGoal;
	}
	return GoalJudge::NotGoal;
}


NodeList RRTConnect::make_path(GoalJudge flag)
{
	if(flag == GoalJudge::SGoal){
		return s_tree.generate_path();
	}
	else if(flag == GoalJudge::GGoal){
		NodeList path = g_tree.generate_path();
		path.reverse();
		return path;
	}
	else if(flag == GoalJudge::Connect){
		return path_concat();
	}

	assert(true);
	return NodeList();
}


bool RRTConnect::extend_limit(Node n1, Node n2)
{
	double threshold = n1.distance(n2);
	if(threshold < 1.0)	return true;
	return false;
}


NodeList RRTConnect::path_concat()
{
	NodeList spath, gpath;
	spath = s_tree.generate_path();
	gpath = g_tree.generate_path();
	gpath.reverse();
	spath.concat(gpath);
	return spath;
}


RRTConnect::RRTConnect()
	:s_tree(), g_tree(), strategy(new DfsCFO()), s_threshold(read_threshold())
{}


NodeList RRTConnect::plan(Node ini, Node fin, State3D goal)
{
	rand_init();

	if(!initialize(ini, fin)){
		std::cout << "Invalid initial value was given." << std::endl;
		return NodeList();
	}

	while(1)
	{
		static int i = 0;	++i;
		if(i>500000)	exit(5963);
		
		// Random sampling and format
		Node Rand = generate_newnode();
		RRTNode opponent_node;
		int opponent_index;
	
		if(i % 10 == 0){
			std::cout << "start conf: " << s_tree.size() << 
			    	   "  goal conf: " << g_tree.size() << std::endl;
		}std::cout << i << ": ";
		
		if(s_tree.size() < g_tree.size()){
			Node newnode = s_tree.format(Rand);
			for(int i=0; i<6; ++i)	std::cout << newnode.node[i] << ", ";	std::cout << std::endl;
			opponent_node = g_tree.get_nearest_node(newnode);
			opponent_index = g_tree.get_nearest_index(newnode);

			if(!sconf_update())	continue;
			RRTNode sconf_newnode = s_tree.back_RRTNode();

			GoalJudge sgj = sconf_goaljudge(goal, sconf_newnode, opponent_node);
			if(sgj != GoalJudge::NotGoal)	return make_path(sgj);

			while(1){
				g_tree.add(opponent_index, newnode); 
				if(!gconf_update())	break;

				GoalJudge ggj = gconf_goaljudge(
						g_tree.back_RRTNode().get_cfree_obj(), 
						s_tree.back_RRTNode().node, opponent_node);
				
				if(ggj != GoalJudge::NotGoal)	return make_path(ggj);

				if(extend_limit(newnode, opponent_node.node)){
					break;
				}
				
				opponent_index = g_tree.get_now_index();
				opponent_node  = g_tree.back_RRTNode();
			}
		}

		else{
			Node newnode = g_tree.format(Rand);
			for(int i=0; i<6; ++i)	std::cout << newnode.node[i] << ", ";	std::cout << std::endl;
			opponent_index = s_tree.get_nearest_index(newnode);
			opponent_node  = s_tree.get_nearest_node(newnode);

			if(!gconf_update())	continue;
			RRTNode gconf_newnode = g_tree.back_RRTNode();

			GoalJudge ggj = gconf_goaljudge(
					g_tree.back_RRTNode().get_cfree_obj(), 
					opponent_node, gconf_newnode);
			
			if(ggj != GoalJudge::NotGoal)	return make_path(ggj);
			
			while(1){
				s_tree.add(opponent_index, newnode);
				if(!sconf_update())	break;
				GoalJudge sgj = sconf_goaljudge(goal, opponent_node, g_tree.back_RRTNode().node);
				if(sgj != GoalJudge::NotGoal)	return make_path(sgj);
			
				if(extend_limit(newnode, opponent_node.node)){
					break;
				}
	
				opponent_index = s_tree.get_now_index();
				opponent_node  = s_tree.back_RRTNode();
			}
		}
	}
	
	return NodeList();
}





void rand_init()
{
	auto seed = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count() % 100000;
	std::srand((unsigned int)seed);
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


double calc_dth2(double th, double goalth)
{
    // Return the value nearer to the goal state
	CSpaceConfig* space = CSpaceConfig::get_instance();
	int sym = space->getsymangle();
    double dif = std::abs(goalth - th);
    return (dif < sym/2) ? dif : sym - dif;
}


double calc_dist(State3D st, State3D goal)
{
    double dy2 = (st.y - goal.y) * (st.y - goal.y);
    double dth2 = calc_dth2(st.th, goal.th);

    double dist = std::sqrt(dy2 + dth2);
    return dist;
}


bool contain_yth(PointCloud pc, State3D goal)
{
    for (int i = 0; i < pc.size(); ++i) 
    if (pc.get(i).y == goal.y && pc.get(i).th == goal.th)    return true;
   
    return false;
}


void print_ICSs(std::vector<PointCloud> pcs)
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

bool duplicate_check(PointCloud subject, std::vector<PointCloud> db)
{
	for(const auto& e : db){
		if(e.exist(subject.get(0)))	return true;
		else continue;
	}
	return false;
}

int read_threshold()
{
    bp::ptree pt;
    read_ini("config/ProblemDefine.ini", pt);
    boost::optional<double> carrier = pt.get_optional<double>("goal.epsilon");
    return carrier.get();
}
