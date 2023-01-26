#include "CFreeICS.h"
#include "Node.h"
#include "TaskSet.h"
#include "Visual.h"
#include "RRT.h"
#include "Problem.h"
#include "pathsmooth.h"
#include "FormClosure.h"
#include "PSO.h"

#include <cassert>
#include <ctime>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <string>
#include <fstream>


#pragma warning(disable : 4996)

std::string get_time_now()
{
	time_t curr_time;
	tm* curr_tm;
	char date[100];
	time(&curr_time);
	curr_tm = localtime(&curr_time);
	strftime(date, 50, "%Y-%B%d-%T", curr_tm);
	return std::string(date);
}


int main(int argc, char* argv[])
{
	std::ofstream log("../ICM_Log/icm.log", std::ios::app);
	std::string fn =  get_time_now();
	log << "\n\n\n" << fn << std::endl;

	TaskSet task;
	task.space_config(10, 10, 5);

    std::cout << "Welcome to Sensorless ICM planner!\n";
    std::cout << "Setting                     -> 1" << std::endl;
    std::cout << "Generate Path(RRT)          -> 2" << std::endl;
    std::cout << "Generate Path(Reverse RRT)  -> 3" << std::endl;
    std::cout << "Generate Path(RRT-Connect)  -> 4" << std::endl;
	std::cout << "Path smoothing              -> 5" << std::endl;
	std::cout << "Debug                       -> 6" << std::endl;
	std::cout << "Calc C_free_ICS             -> 7" << std::endl;
	std::cout << "Form Closure                -> 8" << std::endl;
	std::cout << "Optimization                -> 9" << std::endl;


    int i = 2;
    std::cout << ">";   std::cin >> i;
	assert(i > 0 && i <= 9);

    if (i == 1) {
		log << "--Task setting--" << std::endl;
        TaskSet setting;
        setting.run();
    }

	else if(i == 5){
		log << "--Path smoothing--" << std::endl;
		std::string fn;
		std::cin >> fn;
		NodeList nl = csv_to_nodelist("path/" + fn + ".csv");

		PathSmooth* smoother = new PathSmooth(nl);
		
		NodeList smooth_path = smoother->smooth();
		smooth_path.printIO();	
	}

	else if(i == 6){
		log << "--Debug--" << std::endl;
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
		log << "--Calculate C_free_ICS--" << std::endl;
		Node node(34.338, -49.249, -51.9, 29.55, -46.73, -38.71);
		CFreeICS ics(node);
		std::vector<PointCloud> cics = ics.extract();
		for(int i=0; i<cics.size(); ++i)	std::cout << cics[i] << std::endl;
	}

	else if(i == 8){
//		task.space_config(5, 5, 3);
		std::vector<double> fintmp;
		for(int i=0; i<6; ++i){
			double tmp;	
			std::cout << "Joint " << i+1 << ": ";
			std::cin >> tmp;
			fintmp.push_back(tmp);
		}
		Node fin(fintmp);
//		Node fin(18.99, -29.97, -40.98, 30.71, -30.30, -84.77);
		FormClosure fc(fin);
		fc.close();
		Node fcfin = fc.get_fcangle();
		std::cout << fcfin << std::endl;
	}

	else if(i == 9){
		//task.space_config(5, 5, 3);
		Node fin(50,-60,-20,10, -10, -90);

		PSO opti;
		opti.optimize(fin);
	}


	else{

		Problem* p = nullptr;
	    if (i == 2){
			log << "--Forward RRT exploring--\n";
			p = new Problem(new RRT);
		}
		if (i == 3){
			log << "--Reverse RRT exploring--\n";
			p = new Problem(new RevRRT);
		}
		if (i == 4){
			log << "--RRT-Connect exploring--\n";
			p = new Problem(new RRTConnect);
		}
		
		log.flush();
		NodeList path = p->pathplanning();
		
		long cpu_time = clock();
		double sec = (double)cpu_time / CLOCKS_PER_SEC;
		printf("%f[s]\n", sec);
		log << "Calculation time : " << sec << "[s]\n";

		path.printIO();
		path.print_file(fn);
		log << "Output to " << fn << ".csv\n";

		delete p;
	}
}


