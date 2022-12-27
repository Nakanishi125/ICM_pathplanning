#include <string>

#include "PointCloud.h"
#include "Node.h"

class PathSmooth{
private:
	NodeList orig_path;
	const double alpha = 0.1, beta = 1.5;

	bool robot_update(Node newnode);
	//bool caging_valid(PointCloud prev, Node now, Node aft);
public:
	PathSmooth(NodeList path);

	NodeList smooth();
	bool debug();

};


NodeList csv_to_nodelist(std::string fn);
