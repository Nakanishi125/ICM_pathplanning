#include "TaskSet.h"

#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

using namespace boost::property_tree;

void TaskSet::robotconfig()
{
	int flag = 0;
	ptree pt;

	do {
		std::cout << "Choose hand type (1 -> parallel   2 -> opposite) : ";
		std::cin >> flag;
	} while (flag != 1 && flag != 2);
	
	if (flag == 1) {
		pt.put("origin.rx", 30.01);		pt.put("origin.ry", 0.0);

		pt.put("coordinate.r00", 1);	pt.put("coordinate.r01", 0);
		pt.put("coordinate.r10", 0);	pt.put("coordinate.r11", 1);
	}
	if (flag == 2) {
		pt.put("origin.rx", 30.01);		pt.put("origin.ry", 415.0);

		pt.put("coordinate.r00", -1);	pt.put("coordinate.r01", 0);
		pt.put("coordinate.r10", 0);	pt.put("coordinate.r11", -1);
	}

	pt.put("size.Lh1", 38);		pt.put("size.Lh2", 130);
	pt.put("size.Lh3", 130);	pt.put("size.Lh4", 90);
	pt.put("size.Lw1", 60);		pt.put("size.Lw2", 35);
	pt.put("size.Lw3", 35);		pt.put("size.Lw4", 35);
	pt.put("size.Rh1", 38);		pt.put("size.Rh2", 130);
	pt.put("size.Rh3", 130);	pt.put("size.Rh4", 90);
	pt.put("size.Rw1", 60);		pt.put("size.Rw2", 35);
	pt.put("size.Rw3", 35);		pt.put("size.Rw4", 35);

	pt.put("origin.lx", -30.0);		pt.put("origin.ly", 0.0);
	pt.put("coordinate.l00", 1);	pt.put("coordinate.l01", 0);
	pt.put("coordinate.l10", 0);	pt.put("coordinate.l11", 1);

	write_ini("config/RobotConfig.ini", pt);
	std::cout << std::endl;
}


void TaskSet::problemdef()
{
	ptree pt;
	int th_s1, th_s2, th_s3, th_s4, th_s5, th_s6;
	std::cout << "Input initial state" << std::endl;
	std::cout << "th1: ";	std::cin >> th_s1;
	std::cout << "th2: ";	std::cin >> th_s2;
	std::cout << "th3: ";	std::cin >> th_s3;
	std::cout << "th4: ";	std::cin >> th_s4;
	std::cout << "th5: ";	std::cin >> th_s5;
	std::cout << "th6: ";	std::cin >> th_s6;
	std::cout << std::endl;

	int th_g1, th_g2, th_g3, th_g4, th_g5, th_g6;
	std::cout << "Input final state" << std::endl;
	std::cout << "th1: ";	std::cin >> th_g1;
	std::cout << "th2: ";	std::cin >> th_g2;
	std::cout << "th3: ";	std::cin >> th_g3;
	std::cout << "th4: ";	std::cin >> th_g4;
	std::cout << "th5: ";	std::cin >> th_g5;
	std::cout << "th6: ";	std::cin >> th_g6;
	std::cout << std::endl;

	int x, y, th = 0;
	int e = 0;
	std::cout << "Input goal condition" << std::endl;
	std::cout << "x_goal: ";	std::cin >> x;
	std::cout << "y_goal: ";	std::cin >> y;
	std::cout << "th_goal: ";	std::cin >> th;
	std::cout << "Input threshold: ";	std::cin >> e;

	pt.put("start.th1", th_s1);
	pt.put("start.th2", th_s2);
	pt.put("start.th3", th_s3);
	pt.put("start.th4", th_s4);
	pt.put("start.th5", th_s5);
	pt.put("start.th6", th_s6);
	pt.put("finish.th1", th_g1);
	pt.put("finish.th2", th_g2);
	pt.put("finish.th3", th_g3);
	pt.put("finish.th4", th_g4);
	pt.put("finish.th5", th_g5);
	pt.put("finish.th6", th_g6);
	pt.put("goal.coordx", x);
	pt.put("goal.coordy", y);
	pt.put("goal.coordt", th);
	pt.put("goal.epsilon", e);

	write_ini("config/ProblemDefine.ini", pt);
	std::cout << std::endl;
}


void TaskSet::object()
{
	ptree pt;

	int type = 0;
	int maxi = 3;
	do {
		std::cout << "Choose object:" << std::endl;
		std::cout << "(Rectangle -> 1 , LShape -> 2 , Triangle -> 3): ";

		std::cin >> type;
	} while (type <= 0 || type > maxi);

	pt.put("target.shape", type);

	pt.put("Rectangle.long_side", 100);
	pt.put("Rectangle.short_side", 70);
	pt.put("Rectangle.symmetry", 180);

	pt.put("LShape.long_side", 150);
	pt.put("LShape.short_side", 100);
	pt.put("LShape.long_pro", 40);
	pt.put("LShape.short_pro", 40);
	pt.put("LShape.symmetry", 360);

	pt.put("Triangle.edge1", 100);
	pt.put("Triangle.edge2", 100);
	pt.put("Triangle.edge3", 100);
	pt.put("Triangle.symmetry", 120);

	write_ini("config/ObjectParameter.ini", pt);
	std::cout << std::endl;
}


void TaskSet::space_config(int x, int y, int th)
{
	ptree pt;
	pt.put("range.x", x);
	pt.put("range.y", y);
	pt.put("range.th", th);

	pt.put("top.x", 400);
	pt.put("top.y", 500);
	pt.put("top.th", 360);

	pt.put("bottom.x", -400);
	pt.put("bottom.y", -50);
	pt.put("bottom.th", 0);

	write_ini("config/SpaceConfig.ini", pt);
}


void TaskSet::run()
{
	robotconfig();
	problemdef();
	object();
}
