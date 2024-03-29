#pragma once
#include <stack>

#include "PointCloud.h"
#include "CSpace.h"
#include "Controller.h"
#include "CFreeICS.h"

class CFO
{
public:
	virtual std::vector<PointCloud> extract(PointCloud prev, Node newnode) = 0;

	virtual ~CFO(){}

protected:
	CFO(){}
};


class RasterCFO : public CFO
{
private:
	std::vector<PointCloud> c_freeobj;

	void init()
	{
		c_freeobj.clear();
		c_freeobj.shrink_to_fit();
	}


public:
	RasterCFO(){};
	~RasterCFO(){};

	std::vector<PointCloud> extract(PointCloud prev, Node newnode){
		init();
		CFreeICS cfi(newnode);
		c_freeobj = cfi.extract();

		for(auto itr = c_freeobj.begin(); itr != c_freeobj.end();){
			if(prev.overlap(*itr)){
				++itr;	continue;
			}
			itr = c_freeobj.erase(itr);
		}

		return c_freeobj;
	}
};



class DfsCFO : public CFO
{
private:
	std::vector<PointCloud> c_dfs;
	std::vector<PointCloud> c_del;
	CSpace target;
	PointMarkCloud previous;
	bool cancel;	// if "true", stop explorartion. "false" in the begining.

	void init()
	{
		c_dfs.clear();
		c_dfs.shrink_to_fit();
		target.init();
		previous.init();
		cancel = false;
	}

public:
	DfsCFO()
		:c_dfs(), target(), cancel(false)
	{
	}
	~DfsCFO(){}

	std::vector<PointCloud> extract(PointCloud prev, Node newnode)
	{
		init();
		previous.assign(prev);
		Controller* controller = Controller::get_instance();
		controller->robot_update(newnode);	
		// robot condition is updated, below we can use 
		// no argument function in Controller.h like RintersectS().

		for (int i = 0; i < previous.size(); ++i) {
			if (previous.get_mk(i) == true)	continue;

			previous.toTrue(i);		// Change the state to "True", 
									// which means the point has been explored.
									
			controller->shape_update(previous.get_pt(i));
			if (controller->RintersectS()) {
				continue;
			}
			if (controller->WintersectS()) {
				continue;
			}

			c_dfs.resize(c_dfs.size() + 1);
			int index = target.coord_to_index(previous.get_pt(i));
			target.toTrue(index);
			c_dfs.back().push(previous.get_pt(i));
			cancel = false;
			explore2(previous.get_pt(i));
		}

		return c_dfs;
	}

	bool preprocess(State3D& pt)
	{
		CSpaceConfig* cs = CSpaceConfig::get_instance();
		if(pt.th < 0)	            pt.th = pt.th + (cs->gettop().th + cs->getrange().z);
		if(pt.th > cs->gettop().th)	pt.th = pt.th - (cs->gettop().th + cs->getrange().z);
		int index = target.coord_to_index(pt);
		check_ifexist(pt);

		if (target.elm[index].mk == true)	return false;
		target.toTrue(index);
		return true;
	}

	void explore2(State3D point)
	{	
		Controller* controller = Controller::get_instance();
		for (int i = 1; i <= 26; ++i) {
			std::stack<State3D> stack;
			State3D orig = move(point, i);
			assert(stack.size() == 0);
		//	stack.emplace(orig);

			if (!preprocess(orig))	continue;
			if (edge_judge(orig)) {
				cancel = true;
				continue;
			}

			controller->shape_update(orig);	// shape is updated
			if (controller->RintersectS())	continue;
			if (controller->WintersectS())	continue;

			c_dfs.back().push(orig);
			stack.emplace(orig);

			while (stack.size() != 0)
			{
				State3D pt = stack.top();
				stack.pop();
				for (int i = 1; i <= 26; ++i) {
					State3D next = move(pt, i);

					if (!preprocess(next))	continue;
					if (edge_judge(next)) {
				/////////////////////////////////////////////////////////////////////////////////////
//						PointMarkCloud tmp;
//						for(int x=0; x<previous.size(); ++x){
//							if(!previous.get_mk(x))	tmp.push_back(previous.get_pm(i));
//						}
						//while(tmp.size() != 0){
						//	adjacent_check(tmp);
						//	for(int y=0; y<tmp.size(); ++y){
						//		if(tmp.get_mk(y)){
						//			for(int z=0; z<previous.size(); ++z){
						//				if(previous.get_pt(z) == tmp.get_pt(y))
						//					previous.get_mk(z) = true;
						//			}
						//		}
						//	}
						//	
						//	PointMarkCloud tmp2;
						//	for(int t=0; t<tmp.size(); ++t){
						//		if(!tmp.get_mk(t))	tmp2.push_back(tmp.get_pm(t));
						//	}
						//	tmp = tmp2;
						//}
						//
				/////////////////////////////////////////////////////////////////////////////////////////
						cancel = true;
						continue;
					}

					controller->shape_update(next);	// shape is updated
					if (controller->RintersectS())	continue;
					if (controller->WintersectS())	continue;

					c_dfs.back().push(next);
					stack.emplace(next);
				}
			}
		}
		
		if(cancel)	c_dfs.pop_back();
	}

	State3D move(State3D pt, int dir)
	{
		CSpaceConfig* cs = CSpaceConfig::get_instance();

		if (dir == 1)		return State3D(pt.x - cs->getrange().x, pt.y - cs->getrange().y, pt.th - cs->getrange().z);
		if (dir == 2)		return State3D(pt.x,                    pt.y - cs->getrange().y, pt.th - cs->getrange().z);
		if (dir == 3)		return State3D(pt.x + cs->getrange().x, pt.y - cs->getrange().y, pt.th - cs->getrange().z);
		if (dir == 4)		return State3D(pt.x - cs->getrange().x, pt.y,                    pt.th - cs->getrange().z);
		if (dir == 5)		return State3D(pt.x,                    pt.y,                    pt.th - cs->getrange().z);
		if (dir == 6)		return State3D(pt.x + cs->getrange().x, pt.y,                    pt.th - cs->getrange().z);
		if (dir == 7)		return State3D(pt.x - cs->getrange().x, pt.y + cs->getrange().y, pt.th - cs->getrange().z);
		if (dir == 8)		return State3D(pt.x,                    pt.y + cs->getrange().y, pt.th - cs->getrange().z);
		if (dir == 9)		return State3D(pt.x + cs->getrange().x, pt.y + cs->getrange().y, pt.th - cs->getrange().z);

		if (dir == 10)		return State3D(pt.x - cs->getrange().x, pt.y - cs->getrange().y, pt.th);
		if (dir == 11)		return State3D(pt.x,                    pt.y - cs->getrange().y, pt.th);
		if (dir == 12)		return State3D(pt.x + cs->getrange().x, pt.y - cs->getrange().y, pt.th);
		if (dir == 13)		return State3D(pt.x - cs->getrange().x, pt.y,                    pt.th);
		if (dir == 14)		return State3D(pt.x + cs->getrange().x, pt.y,                    pt.th);
		if (dir == 15)		return State3D(pt.x - cs->getrange().x, pt.y + cs->getrange().y, pt.th);
		if (dir == 16)		return State3D(pt.x,                    pt.y + cs->getrange().y, pt.th);
		if (dir == 17)		return State3D(pt.x + cs->getrange().x, pt.y + cs->getrange().y, pt.th);

		if (dir == 18)		return State3D(pt.x - cs->getrange().x, pt.y - cs->getrange().y, pt.th + cs->getrange().z);
		if (dir == 19)		return State3D(pt.x,                    pt.y - cs->getrange().y, pt.th + cs->getrange().z);
		if (dir == 20)		return State3D(pt.x + cs->getrange().x, pt.y - cs->getrange().y, pt.th + cs->getrange().z);
		if (dir == 21)		return State3D(pt.x - cs->getrange().x, pt.y,                    pt.th + cs->getrange().z);
		if (dir == 22)		return State3D(pt.x,                    pt.y,                    pt.th + cs->getrange().z);
		if (dir == 23)		return State3D(pt.x + cs->getrange().x, pt.y,                    pt.th + cs->getrange().z);
		if (dir == 24)		return State3D(pt.x - cs->getrange().x, pt.y + cs->getrange().y, pt.th + cs->getrange().z);
		if (dir == 25)		return State3D(pt.x,                    pt.y + cs->getrange().y, pt.th + cs->getrange().z);
		if (dir == 26)		return State3D(pt.x + cs->getrange().x, pt.y + cs->getrange().y, pt.th + cs->getrange().z);

		assert(true);
		return State3D();
	}

	void check_ifexist(State3D st)	// If st exists, convert to "true" correspond PointMark of previous C_free_obj
	{
		for (int i = 0; i < (int)previous.size(); ++i) {
			if (previous.get_pt(i) == st) {
				previous.toTrue(i);
				return;
			}
		}
	}

	PointCloud get_cfree_obj()
	{
		return c_dfs[0];
	}

//	void adjacent_check(PointMarkCloud& pmc)
//	{
//		Controller* controller = Controller::get_instance();
//		CSpaceConfig* cs = CSpaceConfig::get_instance();
//		PointMarkCloud backup = pmc;
//
//		for(int i=0; i<pmc.size(); ++i){
//			if (pmc.get_mk(i) == true)	continue;
//			pmc.toTrue(i);	
//
//			controller->shape_update(pmc.get_pt(i));
//			if (controller->RintersectS())	continue;
//			if (controller->WintersectS())	continue;
//
//			for (int d = 1; d <= 26; ++d) {
//				std::stack<State3D> stack;
//				State3D orig = move(pmc.get_pt(i), d);
//				stack.emplace(orig);
//
//				if(orig.th < 0)					orig.th = orig.th + (cs->gettop().th + cs->getrange().z);
//				if(orig.th > cs->gettop().th)	orig.th = orig.th - (cs->gettop().th + cs->getrange().z);
//
//				for (int i = 0; i < (int)pmc.size(); ++i) {
//					if (pmc.get_pt(i) == orig) {
//						pmc.toTrue(i);
//						return;
//					}
//				}
//		
//				if (edge_judge(orig)){
//					pmc = backup;
//					return;
//				}
//
//				controller->shape_update(orig);	// shape is updated
//				if (controller->RintersectS())	continue;
//				if (controller->WintersectS())	continue;
//	
//				if(c_dfs.back().exist(orig))	return;
//				stack.emplace(orig);
//	
//				while (stack.size() != 0)
//				{
//					State3D pt = stack.top();
//					stack.pop();
//					for (int i = 1; i <= 26; ++i) {
//						State3D next = move(pt, i);
//	
//						if(next.th < 0)					next.th = next.th + (cs->gettop().th + cs->getrange().z);
//						if(next.th > cs->gettop().th)	next.th = next.th - (cs->gettop().th + cs->getrange().z);
//						int index = target.coord_to_index(next);
//		
//						for (int i = 0; i < (int)pmc.size(); ++i) {
//							if (pmc.get_pt(i) == next) {
//								pmc.toTrue(i);
//								return;
//							}
//						}
//
//						if (edge_judge(orig)){
//							pmc = backup;
//							return;
//						}
//		
//						controller->shape_update(next);	// shape is updated
//						if (controller->RintersectS())	continue;
//						if (controller->WintersectS())	continue;
//			
//						if(c_dfs.back().exist(next))	return;
//						stack.emplace(next);
//					}
//				}


};





