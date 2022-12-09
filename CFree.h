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


public:
	RasterCFO();
	~RasterCFO();
	std::vector<PointCloud> extract(PointCloud prev, Node newnode);

};



class DfsCFO : public CFO
{
private:
	std::vector<PointCloud> c_dfs;
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
			explore2(previous.get_pt(i));
		}

		return c_dfs;
	}

	bool preprocess(State3D& pt)
	{
		CSpaceConfig* cs = CSpaceConfig::get_instance();
		if(pt.th < 0)	            pt.th = (cs->top.th + cs->range.z) + pt.th;
		if(pt.th > cs->symangle)	pt.th = pt.th - (cs->top.th + cs->range.z);
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
			stack.emplace(orig);

			if (!preprocess(orig))	continue;

			if (edge_judge(orig)) {
				c_dfs.pop_back();
				return;
			}

			controller->shape_update(orig);	// shape is updated
			if (controller->RintersectS())	continue;
			if (controller->WintersectS())	continue;

			c_dfs.back().push(orig);

			while (stack.size() != 0)
			{
				State3D pt = stack.top();
				stack.pop();
				for (int i = 1; i <= 26; ++i) {
					State3D next = move(pt, i);

					if (!preprocess(next))	continue;

					if (edge_judge(next)) {
						c_dfs.pop_back();
						return;
					}

					controller->shape_update(next);	// shape is updated
					if (controller->RintersectS())	continue;
					if (controller->WintersectS())	continue;

					c_dfs.back().push(next);
					stack.emplace(next);
				}
			}
		}
		
	}

	State3D move(State3D pt, int dir)
	{
		CSpaceConfig* cs = CSpaceConfig::get_instance();

		if (dir == 1)		return State3D(pt.x - cs->range.x, pt.y - cs->range.y, pt.th - cs->range.z);
		if (dir == 2)		return State3D(pt.x,               pt.y - cs->range.y, pt.th - cs->range.z);
		if (dir == 3)		return State3D(pt.x + cs->range.x, pt.y - cs->range.y, pt.th - cs->range.z);
		if (dir == 4)		return State3D(pt.x - cs->range.x, pt.y,               pt.th - cs->range.z);
		if (dir == 5)		return State3D(pt.x,               pt.y,               pt.th - cs->range.z);
		if (dir == 6)		return State3D(pt.x + cs->range.x, pt.y,               pt.th - cs->range.z);
		if (dir == 7)		return State3D(pt.x - cs->range.x, pt.y + cs->range.y, pt.th - cs->range.z);
		if (dir == 8)		return State3D(pt.x,               pt.y + cs->range.y, pt.th - cs->range.z);
		if (dir == 9)		return State3D(pt.x + cs->range.x, pt.y + cs->range.y, pt.th - cs->range.z);

		if (dir == 10)		return State3D(pt.x - cs->range.x, pt.y - cs->range.y, pt.th);
		if (dir == 11)		return State3D(pt.x,               pt.y - cs->range.y, pt.th);
		if (dir == 12)		return State3D(pt.x + cs->range.x, pt.y - cs->range.y, pt.th);
		if (dir == 13)		return State3D(pt.x - cs->range.x, pt.y,               pt.th);
		if (dir == 14)		return State3D(pt.x + cs->range.x, pt.y,               pt.th);
		if (dir == 15)		return State3D(pt.x - cs->range.x, pt.y + cs->range.y, pt.th);
		if (dir == 16)		return State3D(pt.x,               pt.y + cs->range.y, pt.th);
		if (dir == 17)		return State3D(pt.x + cs->range.x, pt.y + cs->range.y, pt.th);

		if (dir == 18)		return State3D(pt.x - cs->range.x, pt.y - cs->range.y, pt.th + cs->range.z);
		if (dir == 19)		return State3D(pt.x,               pt.y - cs->range.y, pt.th + cs->range.z);
		if (dir == 20)		return State3D(pt.x + cs->range.x, pt.y - cs->range.y, pt.th + cs->range.z);
		if (dir == 21)		return State3D(pt.x - cs->range.x, pt.y,               pt.th + cs->range.z);
		if (dir == 22)		return State3D(pt.x,               pt.y,               pt.th + cs->range.z);
		if (dir == 23)		return State3D(pt.x + cs->range.x, pt.y,               pt.th + cs->range.z);
		if (dir == 24)		return State3D(pt.x - cs->range.x, pt.y + cs->range.y, pt.th + cs->range.z);
		if (dir == 25)		return State3D(pt.x,               pt.y + cs->range.y, pt.th + cs->range.z);
		if (dir == 26)		return State3D(pt.x + cs->range.x, pt.y + cs->range.y, pt.th + cs->range.z);

		assert(true);
		return State3D();
	}



//	void explore(State3D point)
//	{
//		for (int i = 1; i <= 26; ++i) {
//			zentan(point, i);
//		}
//		return;
//	}
//
//
//	void zentan(State3D pt, int dir)
//	{
//		if (cancel == true)	return;
//	//	if (pt.th < 0)	pt.th = 360 + pt.th;
//	//	if (pt.th > 360)	pt.th = pt.th - 360;
//		if (pt.th < 0)	pt.th = 365 + pt.th;		// to establish consistency to raster scan
//		if (pt.th > 360)	pt.th = pt.th - 365;
//		int index = space.coord_to_index(pt);	// spaceÌÀWÌCfbNXÏ·
//		check_ifexist(pt);
//
//		if (space.get_mk(index) == true)	return;
//		space.toTrue(index);
//		if (edge_judge(pt)) {
//			cancel = true;
//			c_dfs.pop_back();
//			return;
//		}
//
//		Shape* shape = Shape::get_instance();
//		shape->update(pt);
//		if (Collision::RS_intersect(shape))	return;
//		if (Collision::WS_intersect(shape))	return;
//
//		c_dfs.back().push(pt);
//
//		if (dir != 26)		zentan(State3D(pt.x - 10, pt.y - 10, pt.th - 5), 1);
//		if (dir != 25)		zentan(State3D(pt.x     , pt.y - 10, pt.th - 5), 2);
//		if (dir != 24)		zentan(State3D(pt.x + 10, pt.y - 10, pt.th - 5), 3);
//		if (dir != 23)		zentan(State3D(pt.x - 10, pt.y     , pt.th - 5), 4);
//		if (dir != 22)		zentan(State3D(pt.x     , pt.y     , pt.th - 5), 5);
//		if (dir != 21)		zentan(State3D(pt.x + 10, pt.y     , pt.th - 5), 6);
//		if (dir != 20)		zentan(State3D(pt.x - 10, pt.y + 10, pt.th - 5), 7);
//		if (dir != 19)		zentan(State3D(pt.x     , pt.y + 10, pt.th - 5), 8);
//		if (dir != 18)		zentan(State3D(pt.x + 10, pt.y + 10, pt.th - 5), 9);
//
//		if (dir != 17)		zentan(State3D(pt.x - 10, pt.y - 10, pt.th), 10);
//		if (dir != 16)		zentan(State3D(pt.x     , pt.y - 10, pt.th), 11);
//		if (dir != 15)		zentan(State3D(pt.x + 10, pt.y - 10, pt.th), 12);
//		if (dir != 14)		zentan(State3D(pt.x - 10, pt.y     , pt.th), 13);
//		if (dir != 13)		zentan(State3D(pt.x + 10, pt.y     , pt.th), 14);
//		if (dir != 12)		zentan(State3D(pt.x - 10, pt.y + 10, pt.th), 15);
//		if (dir != 11)		zentan(State3D(pt.x     , pt.y + 10, pt.th), 16);
//		if (dir != 10)		zentan(State3D(pt.x + 10, pt.y + 10, pt.th), 17);
//
//		if (dir != 9)		zentan(State3D(pt.x - 10, pt.y - 10, pt.th + 5), 18);
//		if (dir != 8)		zentan(State3D(pt.x     , pt.y - 10, pt.th + 5), 19);
//		if (dir != 7)		zentan(State3D(pt.x + 10, pt.y - 10, pt.th + 5), 20);
//		if (dir != 6)		zentan(State3D(pt.x - 10, pt.y     , pt.th + 5), 21);
//		if (dir != 5)		zentan(State3D(pt.x     , pt.y     , pt.th + 5), 22);
//		if (dir != 4)		zentan(State3D(pt.x + 10, pt.y     , pt.th + 5), 23);
//		if (dir != 3)		zentan(State3D(pt.x - 10, pt.y + 10, pt.th + 5), 24);
//		if (dir != 2)		zentan(State3D(pt.x     , pt.y + 10, pt.th + 5), 25);
//		if (dir != 1)		zentan(State3D(pt.x + 10, pt.y + 10, pt.th + 5), 26);
//	}

//	void dfs(State3D pt, int dir)	// Define recursively
//	{
//		if (cancel == true)	return;
//		if (pt.th < 0 || pt.th > 360)	return;
//		check_ifexist(pt);
//
//		if (edge_judge(pt)) {
//			cancel = true;
//			c_dfs.pop_back();
//			return;
//		}
//
//		Shape* shape = Shape::get_instance();
//		shape->update(pt);
//		if (Collision::RS_intersect(shape))	return;
//		if (Collision::WS_intersect(shape))	return;
//
//		c_dfs.back().push(pt);
//
//		if (dir == 1)								dfs(State3D(pt.x - 10, pt.y - 10, pt.th - 5), 1);
//		if (dir == 1 || dir == 2 || dir == 3)		dfs(State3D(pt.x, pt.y - 10, pt.th - 5), 2);
//		if (dir == 3)								dfs(State3D(pt.x + 10, pt.y - 10, pt.th - 5), 3);
//		if (dir == 1 || dir == 4 || dir == 7)		dfs(State3D(pt.x - 10, pt.y, pt.th - 5), 4);
//		if (dir == 1 || dir == 2 || dir == 3
//			|| dir == 4 || dir == 5 || dir == 6
//			|| dir == 7 || dir == 8 || dir == 9)		dfs(State3D(pt.x, pt.y, pt.th - 5), 5);
//		if (dir == 3 || dir == 6 || dir == 9)		dfs(State3D(pt.x + 10, pt.y, pt.th - 5), 6);
//		if (dir == 7)								dfs(State3D(pt.x - 10, pt.y + 10, pt.th - 5), 7);
//		if (dir == 7 || dir == 8 || dir == 9)		dfs(State3D(pt.x, pt.y + 10, pt.th - 5), 8);
//		if (dir == 9)								dfs(State3D(pt.x + 10, pt.y + 10, pt.th - 5), 9);
//
//		if (dir == 1 || dir == 10 || dir == 18)		dfs(State3D(pt.x - 10, pt.y - 10, pt.th), 10);
//		if (dir == 1 || dir == 2 || dir == 3
//			|| dir == 10 || dir == 11 || dir == 12
//			|| dir == 18 || dir == 19 || dir == 20)	dfs(State3D(pt.x, pt.y - 10, pt.th), 11);
//		if (dir == 3 || dir == 12 || dir == 20)		dfs(State3D(pt.x + 10, pt.y - 10, pt.th), 12);
//		if (dir == 1 || dir == 4 || dir == 7
//			|| dir == 10 || dir == 13 || dir == 15
//			|| dir == 18 || dir == 21 || dir == 24)	dfs(State3D(pt.x - 10, pt.y, pt.th), 13);
//		if (dir == 3 || dir == 6 || dir == 9
//			|| dir == 12 || dir == 14 || dir == 17
//			|| dir == 20 || dir == 23 || dir == 26)		dfs(State3D(pt.x + 10, pt.y, pt.th), 14);
//		if (dir == 7 || dir == 15 || dir == 24)		dfs(State3D(pt.x - 10, pt.y + 10, pt.th), 15);
//		if (dir == 7 || dir == 8 || dir == 9
//			|| dir == 15 || dir == 16 || dir == 17
//			|| dir == 24 || dir == 25 || dir == 26)	dfs(State3D(pt.x, pt.y + 10, pt.th), 16);
//		if (dir == 9 || dir == 17 || dir == 26)		dfs(State3D(pt.x + 10, pt.y + 10, pt.th), 17);
//
//		if (dir == 18)								dfs(State3D(pt.x - 10, pt.y - 10, pt.th + 5), 18);
//		if (dir == 18 || dir == 19 || dir == 20)	dfs(State3D(pt.x, pt.y - 10, pt.th + 5), 19);
//		if (dir == 20)								dfs(State3D(pt.x + 10, pt.y - 10, pt.th + 5), 20);
//		if (dir == 18 || dir == 21 || dir == 24)	dfs(State3D(pt.x - 10, pt.y, pt.th + 5), 21);
//		if (dir == 18 || dir == 19 || dir == 20
//			|| dir == 21 || dir == 22 || dir == 23
//			|| dir == 24 || dir == 25 || dir == 26)	dfs(State3D(pt.x, pt.y, pt.th + 5), 22);
//		if (dir == 20 || dir == 23 || dir == 26)	dfs(State3D(pt.x + 10, pt.y, pt.th + 5), 23);
//		if (dir == 24)								dfs(State3D(pt.x - 10, pt.y + 10, pt.th + 5), 24);
//		if (dir == 24 || dir == 25 || dir == 26)	dfs(State3D(pt.x, pt.y + 10, pt.th + 5), 25);
//		if (dir == 26)								dfs(State3D(pt.x + 10, pt.y + 10, pt.th + 5), 26);
//	}


//	bool edge_judge(State3D pt)
//	{
//		CSpaceConfig* cs = CSpaceConfig::get_instance();
////		if (pt.x == -400 || pt.y == -50)	return true;
////		if (pt.x == 400 || pt.y == 500)		return true;
//		if (pt.x == cs->bottom.x || pt.y == cs->bottom.y)	return true;
//		if (pt.x == cs->top.x || pt.y == cs->top.y)		return true;
//		return false;
//	}

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
};
