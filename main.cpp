#include "CFreeICS.h"
#include "TaskSet.h"
#include "Visual.h"
#include "RRT.h"
#include "Problem.h"
#include "pathsmooth.h"

#include <cassert>
#include <ctime>
#include <iostream>
#include <chrono>
#include <iomanip>

#pragma warning(disable : 4996)

int main(int argc, char* argv[])
{
    std::cout << "Welcome to Sensorless ICM planner!\n";
    std::cout << "Setting                     -> 1" << std::endl;
    std::cout << "Generate Path(RRT)          -> 2" << std::endl;
    std::cout << "Generate Path(Reverse RRT)  -> 3" << std::endl;
    std::cout << "Generate Path(RRT-Connect)  -> 4" << std::endl;
	std::cout << "Path smoothing              -> 5" << std::endl;
	std::cout << "Debug                       -> 6" << std::endl;

    int i = 0;
    std::cout << ">";   std::cin >> i;
	assert(i > 0 && i <= 6);

    if (i == 1) {
        TaskSet setting;
        setting.run();
    }
	else if(i == 5){
		std::string fn;
		std::cin >> fn;
		NodeList nl = csv_to_nodelist("path/" + fn + ".csv");

		PathSmooth* smoother = new PathSmooth(nl);
		
		NodeList smooth_path = smoother->smooth();
		smooth_path.printIO();	
	}
	else if(i == 6){
		std::string fn;
		std::cin >> fn;
		NodeList nl = csv_to_nodelist("path/" + fn + ".csv");

		PathSmooth* smoother = new PathSmooth(nl);
		if(smoother->debug()){
			std::cout << "Valid Route" <<std::endl;
		}
		else{
			std::cout << "Invalid Route" << std::endl;
		}
	}
	else{
		Problem* p = nullptr;
	    if (i == 2)	p = new Problem(new RRT);
		if (i == 3) p = new Problem(new RevRRT);
		if (i == 4) p = new Problem(new RRTConnect);

		NodeList path = p->pathplanning();
		
		long cpu_time = clock();
		double sec = (double)cpu_time / CLOCKS_PER_SEC;
		printf("%f[s]\n", sec);
	
		path.printIO();
		path.print_file("valid4");
		delete p;
	}

	
	Node n(10,20,30,40,50,60);
	CFreeICS* ics = new CFreeICS(n);
	std::vector<PointCloud> pcs = ics->extract();

//    else if (i == 3) {
//        //Node fin(18.8, -21.2, -24.3, 18.9, -14.8, -3);
//        //Node fin(38.1, -39.2, -63.4, 25.7, -41.5, -41.4);
//        Node fin(-44.4, 49.9, 48.7, 64.4, -27.8, -62.6);
//        //Node fin(25, -30, -75, 40, -50, -30);
//        //Node fin(45.7, -37.2, 28.9, -11.5, 27.6, -55.4);
//
//        FormClosure fc(fin);
//        fc.close();
//        Node fcfin = fc.get_fcangle();
//        for (int i = 0; i < Node::dof; ++i) {
//            std::cout << fcfin[i] << ", ";
//        }
//        std::cout << std::endl;
//    }
//    else if (i == 4) {
//        /*Node open(90, 0, 0, 90, 0, 0);
//        FormClosure fc(open);
//        fc.close();
//        Node fcini = fc.get_fcangle();*/
//        Node fcini(32, -39.2, -90, 22.7, -41.5, -44.8);
//        RevRRT rrt(fcini);
//        rrt.planning();
//        rrt.pathprint_IO();
//    }
//    else if (i == 5) {
//        Node ini(25, -30, -75, 30, -30, -20);
//        Node fin(32, -39.2, -90, 22.7, -41.5, -44.8);
//        
//        RRTConnect rrt(ini, fin);
//        NodeList nl = rrt.planning();
//        nl.printIO();
//    }

}

