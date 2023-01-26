#include <fstream>
#include <sstream>

#include "CFree.h"
#include "Controller.h"
#include "pathsmooth.h"
#include "CFreeICS.h"


PathSmooth::PathSmooth(NodeList path)
	:orig_path(path)
{}


NodeList PathSmooth::smooth()
{
	Controller* controller = Controller::get_instance();
	NodeList opt_path = orig_path;

	Node ini = orig_path[0];
	CFreeICS ics(ini);

	for(int i=0; i<6;++i)	std::cout << ini[i] << ", "; std::cout << std::endl;
	std::vector<PointCloud> init_CFree = ics.extract();
	int num = 0;
	for (const auto& cls : init_CFree) {
		std::cout << num;	std::cout << ":" << cls.size() << std::endl;
		for (int i = 0; i < (int)cls.size(); ++i) {
			std::cout << "[";	std::cout << cls.get(i).x;	std::cout << ",";
			std::cout << cls.get(i).y;	std::cout << ",";	std::cout << cls.get(i).th;	std::cout << "],";
		//	if (i > 4)	break;
		}
		std::cout << std::endl << std::endl;
		++num;
	}

	std::cout << "Select the cluster:";
	int index = -1;
	std::cin >> index;
	assert(0 <= index && index < (int)init_CFree.size());

	int cnt = 0, iters_num = 10000;
	double tolerance = 0.1;
	double error = 1.0;	// All value is OK except the value lower than 0.1
	
//	while(error > tolerance || cnt < iters_num){
	while(cnt < iters_num){
		PointCloud pre_cfo = init_CFree[index];
		error = 0.0;
		// initial and final point is fixed.
		for(int i=1; i<orig_path.size()-1; ++i){
			NodeList pre_path = opt_path;
			//opt_path[i] = opt_path[i] - alpha*(opt_path[i] - orig_path[i]);
			//opt_path[i] = opt_path[i] - beta*(2*opt_path[i] - opt_path[i-1] - opt_path[i+1]);
			for(int n=0; n<Node::dof; ++n){
				if(opt_path[i][n] > 90)		opt_path[i][n] = 90;
				if(opt_path[i][n] < -90)	opt_path[i][n] = -90;
			}
			std::cout << "before path: ";
			for(int elm=0; elm<Node::dof; ++elm)	std::cout << pre_path[i][elm] << ", ";	std::cout << std::endl;
			std::cout << "after  path: ";
			for(int elm=0; elm<Node::dof; ++elm)	std::cout << opt_path[i][elm] << ", ";	std::cout << std::endl;
			error += opt_path[i].norm(pre_path[i]);

			if(!robot_update(opt_path[i])){
				opt_path[i] = pre_path[i];
				break;
			}

			// caging_valid start
			DfsCFO dfs;
			std::vector<PointCloud> cfo_now = dfs.extract(pre_cfo, opt_path[i]);
			if((int)cfo_now.size() != 1){
				opt_path[i] = pre_path[i];
				std::vector<PointCloud> cfo_org = dfs.extract(pre_cfo, pre_path[i]);
				assert(cfo_org.size() == 1);
				pre_cfo = cfo_org[0];
				continue;
			}
			
			std::vector<PointCloud> cfo_aft = dfs.extract(cfo_now[0], opt_path[i+1]);
			if((int)cfo_aft.size() != 1){
				opt_path[i] = pre_path[i];
				std::vector<PointCloud> cfo_org = dfs.extract(pre_cfo, pre_path[i]);
				assert(cfo_org.size() == 1);
				pre_cfo = cfo_org[0];
				continue;
			}

			pre_cfo = cfo_now[0];
			// caging_valid end
		}
		std::cout << "Loop End" << std::endl;
		++cnt;
		std::cout << "No. " << cnt << std::endl;
		std::cout << "  diff: " << error << std::endl;
	}

	return opt_path;
}


bool PathSmooth::robot_update(Node newnode)
{
	Controller* controller = Controller::get_instance();
	controller->robot_update(newnode);

	if (controller->RintersectR(newnode)) {
		return false;
	}
	if (controller->RintersectW(newnode)) {
		return false;
	}
	return true;
}


NodeList csv_to_nodelist(std::string fn)
{
	NodeList nl;
	std::string tmp_line, tmp_str;
	std::vector<double> node(6);
	std::ifstream ifs(fn);

	while(getline(ifs, tmp_line)){
		std::istringstream i_stream(tmp_line);
		int index = 0;
		while(getline(i_stream, tmp_str, ',')){
			node[index] = std::stod(tmp_str);
			++index;
			if(index > 5)	break;
		}
		nl.push_back(node);
	}
	return nl;
}


bool PathSmooth::debug()
{
	Controller* controller = Controller::get_instance();
	std::string fn = "../icmlog/debug.txt";
	std::ofstream ofs(fn, std::ios::app);

	Node ini = orig_path[0];
	CFreeICS ics(ini);

	std::cout << ini << std::endl;
	std::vector<PointCloud> init_CFree = ics.extract();
	int num = 0;
	for (const auto& cls : init_CFree) {
		std::cout << num;	std::cout << ":" << cls.size() << std::endl;
		for (int i = 0; i < (int)cls.size(); ++i) {
			std::cout << "[";	std::cout << cls.get(i).x;	std::cout << ",";
			std::cout << cls.get(i).y;	std::cout << ",";	std::cout << cls.get(i).th;	std::cout << "],";
		//	if (i > 4)	break;
		}
		std::cout << std::endl << std::endl;
		++num;
	}

	std::cout << "Select the cluster:";
	int index = -1;
	std::cin >> index;
	assert(0 <= index && index < (int)init_CFree.size());

	PointCloud pre_cfo = init_CFree[index];

	for(int i=1; i<orig_path.size(); ++i){
		std::cout << i << ": " << orig_path[i] << " -> ";
		ofs << i << ": " << orig_path[i] << std::endl;
		if(!robot_update(orig_path[i])){
			return false;
		}

		DfsCFO dfs;
		std::vector<PointCloud> cfo_now = dfs.extract(pre_cfo, orig_path[i]);
		for(int i=0; i<cfo_now.size(); ++i){
			ofs << cfo_now[i] << std::endl;
		}

		if((int)cfo_now.size() != 1){
			return false;
		}
		std::cout << cfo_now[0].size() << std::endl;	
		pre_cfo = cfo_now[0];
	}
	return true;
}
