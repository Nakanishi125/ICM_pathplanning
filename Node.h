#pragma once

#include <vector>
#include <string>

class HalfNode
{
private:
	std::vector<double> hnode;

public:
	const static int dof = 3;
	HalfNode(double ang1, double ang2, double ang3);

	double get_element(int n);
};


struct Node
{
	std::vector<double> node;

	const static int dof = 6;

	Node();
	Node(double e1, double e2, double e3, double e4, double e5, double e6);
	Node(std::vector<double> nd);

	double get_element(int n); 
	double get_element(int n) const;
	HalfNode get_langles();
	HalfNode get_rangles();
	double get_absangle(int index) const;

	double distance(const Node& other);
	Node normalize(const Node& other);		// “®‚­‚Ì‚Í*this
	Node unitmove(const Node& other);	// from *this to other

	double& operator [](int dof) { return node[dof]; }
};


class NodeList
{
private:
	std::vector<Node> serial_node;

public:
	NodeList();

	void push_back(Node add_node);
	void reverse();
	void printIO();
	void print_file(std::string fn);

	int size() const { return (int)serial_node.size(); }
	Node get(int i) const { return serial_node[i]; }

	void concat(const NodeList& other);	// this ‚ÌŒã‚É other
};
