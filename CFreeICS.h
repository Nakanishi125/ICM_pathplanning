#pragma once
#include <cassert>

#include "CSpace.h"
#include "Node.h"
#include "Controller.h"
#include "Labeling.h"


bool edge_judge(State3D pt);
bool contain_edge(PointCloud pc, int edge);


class CFreeICS
{
private:
	std::vector<PointCloud> c_free_ics;
	CSpace cspace;
	Node nownode;

public:
	CFreeICS(Node node)
		:c_free_ics(), cspace(), nownode(node){}

	std::vector<PointCloud> extract()
	{
		Controller* controller = Controller::get_instance();
		CSpaceConfig* conf = CSpaceConfig::get_instance();	
		controller->robot_update(nownode);

		for(int i=0; i<cspace.size(); ++i){
			State3D pos = cspace.elm[i].pt;
			controller->shape_update(pos);
			if(controller->RintersectS())	    cspace.toFalse(i);
			else if(controller->WintersectS())	cspace.toFalse(i);
			else                                cspace.toTrue(i);
		}

		Labeling* label = new Labeling(cspace);
		int clust = label->labeling3D();
		c_free_ics.resize(clust);

		for (int ix = 0; ix < conf->getnumx(); ++ix)
			for (int iy = 0; iy < conf->getnumy(); ++iy)
				for (int ith = 0; ith < conf->getnumth(); ++ith) 
					distribute(label, ix, iy, ith);

		delete label;

		int exit_flag = 0;
		for(auto it=c_free_ics.begin(); it!=c_free_ics.end();){
			exit_flag = 0;
			for(int i=0; i<(*it).size(); ++i){
				if(edge_judge((*it).elm[i])){
					it = c_free_ics.erase(it);
					exit_flag = 1;
					break;
				}
			}
			if(exit_flag == 1)	continue;
			++it;
		}

		if(c_free_ics.size() == 0)	return c_free_ics;

		merge(conf->getsymangle());
		return c_free_ics;
	}

	void distribute(Labeling* lbel, int ix, int iy, int ith)
	{
		CSpaceConfig* conf = CSpaceConfig::get_instance();	
		State3D btm = conf->getbottom();
		Vector3D<int> rng = conf->getrange();
		int area = lbel->get_label(ix, iy, ith);
		if (area != 0)
			c_free_ics[area - 1].push(State3D(btm.x + rng.x*ix, btm.y + rng.y*iy, btm.th + rng.z*ith));
		
	}

	void merge(int edge)
	{
		std::vector<PointCloud> subject;
		std::vector<PointCloud> rest;

		for(const auto& e : c_free_ics){
			if(contain_edge(e, edge))	subject.push_back(e);
			else                        rest.push_back(e);
		}
	
		PCMerge mg(subject, edge);
		mg.merge();
	
		c_free_ics = mg.getter();
		c_free_ics.insert(c_free_ics.end(), rest.begin(), rest.end());

	}


};


