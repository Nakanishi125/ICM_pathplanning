#include "ProblemDefinition.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

namespace bp = boost::property_tree;

ProblemDefinition::ProblemDefinition()
	:initial(read_initial()), gj()
{}


//bool ProblemDefinition::goal_judge(PointCloud pc)
//{
////	if (gj.judge(pc))	return true;
//	if (gj.judge_simple(pc))	return true;
//	return false;
//}
//
//
//bool ProblemDefinition::semigoal_judge(PointCloud pc)
//{
////	if (gj.semijudge(pc))	return true;
//	if (gj.semijudge_simple(pc))	return true;
//	return false;
//}


int ProblemDefinition::judge_once(PointCloud pc)
{
	return gj.judge_once(pc);
}


Node read_initial()
{
	bp::ptree pt;
	int th1, th2, th3, th4, th5, th6 = 0;
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

	//return Node(th1, th2, th3, th4, th5, th6);
	return Node(25.6432, -29.8511, -13.9475, 34.3596, -25.2216, -27.6198);
}