#pragma once

#include "Node.h"
#include "icmMath.h"
#include "PointCloud.h"


class FormClosure
{
private:
	const Node ini;
	State3D goal;
	Node fcang;

	int nind_to_lind(int nind);		// Function that converts Node index to Robot link index
	int lind_to_nind(int lind);		// Ditto but reverse

public:
	FormClosure(Node _ini);

	void init();	// Initialize Robot and Shape configuration

	void simple_narrow(std::vector<int> order);			// Function to implement pseudo Form Closure according to Maeda Prof. algorithm (refer GM0607)
	int explore(int nind);				// Function to move robot-hands toward minus direction in the order of root to tip and get intersection angle
	int reverse_explore(int nind);		// This function is called when explore() does not find intersection angle. This function makes robot-hands move toward plus direction

	void close();					// The manager of this class.


	Node get_fcangle();
	double fc_eval(PointCloud pc);
};


