#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

#include "Problem.h"

namespace bp = boost::property_tree;


Problem::Problem(Planner* solver)
	:solver(solver), 
	 initial(read_initial()), 
	 finish(read_finish()),
	 goal(read_goal()) 
{}

NodeList Problem::pathplanning()
{
	std::ofstream log("icm.log", std::ios::app);
	log << "initial robot state : " << initial << std::endl;
	log << "finish  robot state : " << finish << std::endl;
	log << "object goal state : " << "[" << goal.x << "," << goal.y << "," << goal.th << "]" << std::endl;

	return solver->plan(initial, finish, goal);
}


Node read_initial()
{
	bp::ptree pt;
	int th1, th2, th3, th4, th5, th6;
	read_ini("config/ProblemDefine.ini", pt);
	boost::optional<int> carrier = pt.get_optional<int>("start.th1");
	th1 = carrier.get();
	carrier = pt.get_optional<int>("start.th2");
	th2 = carrier.get();
	carrier = pt.get_optional<int>("start.th3");
	th3 = carrier.get();
	carrier = pt.get_optional<int>("start.th4");
	th4 = carrier.get();
	carrier = pt.get_optional<int>("start.th5");
	th5 = carrier.get();
	carrier = pt.get_optional<int>("start.th6");
	th6 = carrier.get();

	return Node(th1, th2, th3, th4, th5, th6);
}


Node read_finish()
{
	bp::ptree pt;
	int th1, th2, th3, th4, th5, th6;
	read_ini("config/ProblemDefine.ini", pt);
	boost::optional<int> carrier = pt.get_optional<int>("finish.th1");
	th1 = carrier.get();
	carrier = pt.get_optional<int>("finish.th2");
	th2 = carrier.get();
	carrier = pt.get_optional<int>("finish.th3");
	th3 = carrier.get();
	carrier = pt.get_optional<int>("finish.th4");
	th4 = carrier.get();
	carrier = pt.get_optional<int>("finish.th5");
	th5 = carrier.get();
	carrier = pt.get_optional<int>("finish.th6");
	th6 = carrier.get();

	return Node(th1, th2, th3, th4, th5, th6);
}

State3D read_goal()
{
    bp::ptree pt;
    read_ini("config/ProblemDefine.ini", pt);
    boost::optional<int> carrier = pt.get_optional<int>("goal.coordx");
    int x = carrier.get();
    carrier = pt.get_optional<int>("goal.coordy");
    int y = carrier.get();
    carrier = pt.get_optional<int>("goal.coordt");
    int th = carrier.get();

    return State3D(x, y, th);
}



