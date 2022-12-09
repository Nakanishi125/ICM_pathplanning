#pragma once

#include <vector>

#include "Node.h"
#include "PointCloud.h"


// To unify the datatype with RRTNodeList
// std::vector�̃��b�p�[�N���X
class NeighborList
{
private:
	std::vector<int> list;

public:
	NeighborList();
	NeighborList(std::vector<int> old);

	int size();
	void push_back(int index);
	void pop_back();

	int get(int i);

	std::vector<int> nodenum_order(int end);
};


struct RRTNode
{
	Node node;		// ���{�b�g�U���R�x�̊֐ߊp�x��RRT�̂P�m�[�h�ɑ���
	std::vector<PointCloud> cfree_obj;		//�@C_free_ics�̓��C�Ó��Ɣ��肳�ꂽ�̈�
	std::vector<PointCloud> cfree_del;		//  C_free_ics�̓��C�Ó��ł͂Ȃ��Ɣ��肳�ꂽ�̈�

	RRTNode();
	RRTNode(Node _n);
	RRTNode(Node _n, PointCloud pc);
	RRTNode(Node _n, std::vector<PointCloud> pcs, std::vector<PointCloud> del);

	double distance(const Node& other);
	double distance(const RRTNode& other);

	// getter
	Node getNode();
	PointCloud pc();

	std::vector<PointCloud> get_cfree_obj();
	std::vector<PointCloud> get_cfree_del();
};


class RRTTree 
{
private:
	std::vector<RRTNode> graph;
	NeighborList nl;

public:
	RRTTree(Node ini, int origin);
	RRTTree();

	Node format(Node targ);
	Node add(int root_index, Node leaf);

	void push_back(Node targ, int oya);
	void push_back(RRTNode targ, int oya);
	void pop_back();
	void replace(RRTNode targ);
	int size();

	RRTNode get_nearest_node(Node targ);
	RRTNode get_RRTNode(int index);	
	RRTNode back_RRTNode();
	RRTNode get_parentRRTNode(int index);
	RRTNode back_parentRRTNode();
	 
	int get_parent_index(int index);		
	int get_nearest_index(Node targ);
	int get_now_index();

	NodeList generate_path();
};
