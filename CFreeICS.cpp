

#include "CFreeICS.h"


bool edge_judge(State3D pt)
{
	CSpaceConfig* cs = CSpaceConfig::get_instance();
	if (pt.x == cs->bottom.x || pt.y == cs->bottom.y)	return true;
	if (pt.x == cs->top.x || pt.y == cs->top.y)		return true;
	return false;
}


bool contain_edge(PointCloud pc, int edge)
{
	for (int i = 0; i < pc.size(); ++i) {
		if(pc.elm[i].th == 0 || pc.elm[i].th == edge)	return true;
	}
	return false;
}
