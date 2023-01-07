#include "CFreeICS.h"
#include "Node.h"
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
	std::cout << "Calc C_free_ICS             -> 7" << std::endl;


    int i = 0;
    std::cout << ">";   std::cin >> i;
	assert(i > 0 && i <= 7);

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
		fn = "path/" + fn + ".csv";
		NodeList nl = csv_to_nodelist(fn);
		PathSmooth* ps = new PathSmooth(nl);
		bool valid = ps->debug();
		if(valid){
			std::cout << "Valid Route" <<std::endl;
		}
		else{
			std::cout << "Invalid Route" << std::endl;
		}
	}
	else if(i == 7){
		Node node(5.63529, -2.53159, -1.09739, 10.5113, 30.4022, -64.6151);
		CFreeICS ics(node);
		std::vector<PointCloud> cics = ics.extract();
		for(int i=0; i<cics.size(); ++i)	std::cout << cics[i] << std::endl;
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

}

