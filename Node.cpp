#include <cmath>
#include <iostream>
#include <cassert>
#include <fstream>

#include "Node.h"


HalfNode::HalfNode(double ang1, double ang2, double ang3)
{
	hnode.resize(3);
	hnode[0] = RobotDouble(ang1);
	hnode[1] = RobotDouble(ang2);
	hnode[2] = RobotDouble(ang3);
}

HalfNode::HalfNode(RobotDouble ang1, RobotDouble ang2, RobotDouble ang3)
{
	hnode.resize(3);
	hnode[0] = ang1;
	hnode[1] = ang2;
	hnode[2] = ang3;
}

double HalfNode::get_element(int num)
{
	double angle = 0;
	for (int i = 0; i <= num; ++i) {
		angle += hnode[i].value();
	}
	return angle;
}




Node::Node()
	:node()
{
	RobotDouble zero(0);
	node = {zero, zero, zero, zero, zero, zero};
}


Node::Node(double e1, double e2, double e3, double e4, double e5, double e6)
	: node()
{
	node.resize(6);
	node[0] = RobotDouble(e1);	node[1] = RobotDouble(e2);	node[2] = RobotDouble(e3);
	node[3] = RobotDouble(e4);	node[4] = RobotDouble(e5);	node[5] = RobotDouble(e6);
}


Node::Node(std::vector<double> nd)
	: node()
{
	node.resize(6);
	node[0] = RobotDouble(nd[0]);node[1] = RobotDouble(nd[1]);node[2] = RobotDouble(nd[2]);
	node[3] = RobotDouble(nd[3]);node[4] = RobotDouble(nd[4]);node[5] = RobotDouble(nd[5]);
}


Node::Node(std::vector<RobotDouble> nd)
	: node(nd)
{
	assert(nd.size() == 6);
}


double Node::get_element(int n) 
{
	return node[n].value();
}
void Node::set_element(int dof, double d) 
{
	node[dof] = RobotDouble(d);
}
double Node::get_element(int n) const
{
	return node[n].value();
}

HalfNode Node::get_langles()
{
	HalfNode hn(node[0], node[1], node[2]);
	return hn;
}


HalfNode Node::get_rangles()
{
	HalfNode hn(node[3], node[4], node[5]);
	return hn;
}


double Node::get_absangle(int index) const
{
	if (index == 0)	return node[0].value();
	if (index == 1)	return node[0].value() + node[1].value();
	if (index == 2)	return node[0].value() + node[1].value() + node[2].value();
	if (index == 3)	return node[3].value();
	if (index == 4) return node[3].value() + node[4].value();
	if (index == 5)	return node[3].value() + node[4].value() + node[5].value();
	return 999;
}


double Node::distance(const Node& other)
{
	double dist = 0.0;
	for (int i = 0; i < 6; ++i) {
		dist += (this->node[i].value() - other.node[i].value()) 
		  	  * (this->node[i].value() - other.node[i].value());
	}
	return std::sqrt(dist);
}


// otherÇ∆ÇÃãóó£Ç™1Ç…Ç»ÇÈÇÊÇ§Ç…*thisÇìÆÇ©Ç∑ÅCotherÇÕå≈íË
Node Node::normalize(const Node& other)
{
	double dist = distance(other);
	std::vector<double> new_node(6);
	for (int i = 0; i < Node::dof; ++i) {
		double trans = (node[i].value() - other.node[i].value()) / dist;
		new_node[i] = other.node[i].value() + trans;
	}

	return Node(new_node);
}


Node Node::unitmove(const Node& other)
{
	double dist = distance(other);
	if (dist < 1.0)	return *this;

	std::vector<double> new_node(6);
	for (int i = 0; i < Node::dof; ++i) {
		double trans = (other.node[i].value() - node[i].value()) / dist;
		new_node[i] = node[i].value() + trans;
	}

	return Node(new_node);
}


Node Node::operator+(const Node& other)	const
{
	return Node(node[0].value() + other.node[0].value(),
			    node[1].value() + other.node[1].value(),
			    node[2].value() + other.node[2].value(),
			    node[3].value() + other.node[3].value(),
			    node[4].value() + other.node[4].value(),
			    node[5].value() + other.node[5].value());
}

Node Node::operator-(const Node& other)	const
{
	return Node(node[0].value() - other.node[0].value(),
			    node[1].value() - other.node[1].value(),
			    node[2].value() - other.node[2].value(),
			    node[3].value() - other.node[3].value(),
			    node[4].value() - other.node[4].value(),
			    node[5].value() - other.node[5].value());
}

Node Node::operator*(int r)	const
{
	return Node(r*node[0].value(), r*node[1].value(), r*node[2].value(),
			    r*node[3].value(), r*node[4].value(), r*node[5].value());
}

Node operator*(int r, const Node& other)
{
	return Node(r*other.node[0].value(), r*other.node[1].value(), r*other.node[2].value(),
			    r*other.node[3].value(), r*other.node[4].value(), r*other.node[5].value());
}

double Node::norm(const Node& other)
{
	double sqr = (node[0].value()-other.node[0].value())*(node[0].value()-other.node[0].value()) 
				+(node[1].value()-other.node[1].value())*(node[1].value()-other.node[1].value()) 
				+(node[2].value()-other.node[2].value())*(node[2].value()-other.node[2].value()) 
				+(node[3].value()-other.node[3].value())*(node[3].value()-other.node[3].value()) 
				+(node[4].value()-other.node[4].value())*(node[4].value()-other.node[4].value()) 
				+(node[5].value()-other.node[5].value())*(node[5].value()-other.node[5].value());
	return std::sqrt(sqr);
}








NodeList::NodeList()
	:elm()
{}

void NodeList::push_back(Node add_node)
{
	elm.push_back(add_node);
}

void NodeList::printIO()
{
	for (int i = 0; i < (int)elm.size(); ++i) {
		for (int dof = 0; dof < 6; ++dof) {
			std::cout << elm[i].get_element(dof) << ", ";
		}
		std::cout << std::endl;
	}
}

void NodeList::print_file(std::string fn)
{
	std::string fp = "path/" + fn + ".csv";
	std::ofstream file(fp, std::ios::app);
	for (int i = 0; i < (int)elm.size(); ++i) {
		for (int dof = 0; dof < 6; ++dof) {
			file << elm[i].get_element(dof) << ", ";
		}
		file << std::endl;
	}
	file.close();
}

void NodeList::reverse()
{
	const std::vector<Node> copy(elm);
	const int nlsize = (int)copy.size();

	for (int i = 0; i < nlsize; ++i) {
		elm[i] = copy[nlsize - 1 - i];
	}
}


void NodeList::concat(const NodeList& other)
{
	for (int i = 0; i < other.size(); ++i) {
		elm.push_back(other.get(i));
	}
}

std::ostream& operator<<(std::ostream& out, const RobotDouble &rd){
	out << rd.value();	return out;
}

std::ostream& operator<<(std::ostream& out, const Node &nd)
{
	out << "[" 
		<< nd.get_absangle(0) << ", " << nd.get_element(1) << ", "
	    << nd.get_absangle(2) << ", " << nd.get_element(3) << ", "
	    << nd.get_absangle(4) << ", " << nd.get_element(5) << "]";
	return out;
}


