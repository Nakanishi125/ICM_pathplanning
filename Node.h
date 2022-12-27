#pragma once

#include <vector>
#include <string>
#include <sstream>
#include <iomanip>


struct RobotDouble
{
	std::string val;

	RobotDouble()
		:val("")
	{
	}
	RobotDouble(double _val){
		std::ostringstream oss;
		oss << std::fixed << std::setprecision(3) << _val;
		val = oss.str();
	}

	double value() const {
		return std::stod(val);
	}

};

std::ostream& operator<<(std::ostream& out, const RobotDouble &rd);

struct HalfNode
{
	std::vector<RobotDouble> hnode;

	const static int dof = 3;
	HalfNode(double ang1, double ang2, double ang3);
	HalfNode(RobotDouble ang1, RobotDouble ang2, RobotDouble ang3);

	double get_element(int n);
};


struct Node
{
	std::vector<RobotDouble> node;

	const static int dof = 6;

	Node();
	Node(double e1, double e2, double e3, double e4, double e5, double e6);
	Node(std::vector<double> nd);
	Node(std::vector<RobotDouble> nd);

	double get_element(int n); 
	void set_element(int dof, double d);
	double get_element(int n) const;
	HalfNode get_langles();
	HalfNode get_rangles();
	double get_absangle(int index) const;

	double distance(const Node& other);
	Node normalize(const Node& other);		// �����̂�*this
	Node unitmove(const Node& other);	// from *this to other

	Node operator+(const Node& other)	const;
	Node operator-(const Node& other)	const;
	Node operator*(int r)	const;
	friend Node operator*(int r, const Node& other);
	double norm(const Node& other);
};

std::ostream& operator<<(std::ostream& out, const Node &nd);

struct NodeList
{
	std::vector<Node> elm;

	NodeList();

	void push_back(Node add_node);
	void reverse();
	void printIO();
	void print_file(std::string fn);

	int size() const { return (int)elm.size(); }
	Node get(int i) const { return elm[i]; }
	Node& operator[](int i)	{ return elm[i]; }

	void concat(const NodeList& other);	// this �̌�� other
};
