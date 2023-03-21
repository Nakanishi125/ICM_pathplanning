#include "CFreeICS.h"
#include "Node.h"
#include "TaskSet.h"
#include "Visual.h"
#include "RRT.h"
#include "Problem.h"
#include "pathsmooth.h"
#include "FormClosure.h"
#include "PSO.h"
#include "TShape.h"

#include <cassert>
#include <ctime>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <string>
#include <fstream>


//#pragma warning(disable : 4996)

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

    std::cout << "Welcome to Sensorless ICM planner!\n";
	std::cout << "----------------------------------------------\n";
    std::cout << "Setting                     -> 1" << std::endl;
    std::cout << "Generate Path(RRT)          -> 2" << std::endl;
    std::cout << "Generate Path(Reverse RRT)  -> 3" << std::endl;
    std::cout << "Generate Path(RRT-Connect)  -> 4" << std::endl;
	std::cout << "Path smoothing              -> 5" << std::endl;
	std::cout << "Debug                       -> 6" << std::endl;
	std::cout << "Calc C_free_ICS             -> 7" << std::endl;
	std::cout << "Form Closure                -> 8" << std::endl;
	std::cout << "Optimization                -> 9" << std::endl;
	std::cout << "----------------------------------------------\n";

    int i = 0;
    std::cout << ">";   std::cin >> i;
	assert(i > 0 && i <= 9);

    if (i == 1) {
		log << "--Task Setting--" << std::endl;
        TaskSet setting;

		int opt = 0;
		std::cout << "-----------------------------------------\n";
		std::cout << "Check current config         -> 1\n";	
		std::cout << "Set the shape                -> 2\n";
		std::cout << "Set the Robot angle config   -> 3\n";
		std::cout << "Set the goal condition       -> 4\n";
		std::cout << "Set the all config           -> 5\n\n";
		std::cout << "Set the hand config          -> 6\n";
		std::cout << "Set the discretized variable -> 7\n";
		std::cout << "-----------------------------------------\n";
		std::cout << ">";	std::cin >> opt;


		if(opt == 1)		setting.check();
		else if(opt == 2)	setting.set_shape();
		else if(opt == 3)	setting.set_robotangle();
		else if(opt == 4)	setting.set_goal();
		else if(opt == 5)	setting.set_all();
		else if(opt == 6)	setting.set_handtype();
		else if(opt == 7)	setting.set_discretization();
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
		fn = "../ICM_Log/path/" + fn + ".csv";
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
//		task.space_config(5, 5, 1);
		Node node(21.2,-23.8,-84.6,18.88,-40.2,-13.922);

		CFreeICS ics(node);
		std::vector<PointCloud> cics = ics.extract();
		for(int i=0; i<(int)cics.size(); ++i){
			std::cout << i << ":\n" << cics[i] << std::endl;
		}

		std::cout << "Select the cluster No." << std::endl;
		std::cout << "-> ";
		int cls = 0;	std::cin >> cls;
		PointCloud pc = cics[cls];
		std::ofstream ofs("../cluster.csv");
		for(int i=0; i<pc.size(); ++i){
			ofs << pc.get(i).x << "," << pc.get(i).y << "," << pc.get(i).th << std::endl;
		}

	}

	else if(i == 8){
//		task.space_config(5, 5, 1);
//		std::vector<double> fintmp;
//		for(int i=0; i<6; ++i){
//			double tmp;	
//			std::cout << "Joint " << i+1 << ": ";
//			std::cin >> tmp;
//			fintmp.push_back(tmp);
//		}
//		Node fin(fintmp);
		Node fin(34.338, -49.249, -51.9, 29.55, -46.73, -38.71);
		FormClosure fc(fin);
		fc.close();
		Node fcfin = fc.get_fcangle();
		std::cout << "Closed hand config : " << fcfin << std::endl;
	}

	else if(i == 9){
//		task.space_config(5, 5, 3);
		Node fin(0, 0, 0, 0, 0, 0);

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


