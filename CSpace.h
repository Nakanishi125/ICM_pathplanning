#pragma once

#include <cassert>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

#include "PointCloud.h"
#include "icmMath.h"


namespace bp = boost::property_tree;


int read_symangle();
State3D read_top();
State3D read_bottom();
Vector3D<int> read_range();


class CSpaceConfig
{
public:
	const State3D top, bottom;
	const Vector3D<int> range;
	const int symangle;
	int numx, numy, numth;

	static CSpaceConfig* get_instance();

private:
	CSpaceConfig();
	static CSpaceConfig* instance;
};


// To deal with whole discretization C-Space
struct CSpace
{
	std::vector<PointMark> elm;

	CSpace()
	{
		CSpaceConfig* cs = CSpaceConfig::get_instance();
		elm.resize(cs->numx * cs->numy * cs->numth);

		int debug = 0;
		for (int i = 0; i < cs->numx; ++i) {
			for (int j = 0; j < cs->numy; ++j) {
				for (int k = 0; k < cs->numth; ++k) {
					State3D pos(cs->bottom.x + cs->range.x * i,
						        cs->bottom.y + cs->range.y * j,
						        cs->bottom.th + cs->range.z * k);

					int index = i*cs->numy*cs->numth + j*cs->numth + k;
					elm[index] = PointMark(pos, false);
					debug++;
				}
			}
		}
		assert(debug == cs->numx * cs->numy * cs->numth);
	}

	void init()
	{
		for(int i=0; i<elm.size(); ++i){
			elm[i].mk = false;
		}
	}

	int size(){
		return (int)elm.size();
	}

	State3D get_pt(int i){
		return elm[i].pt;
	}

	void toFalse(int i)
	{
		elm[i].toFalse();
	}
	void toTrue(int i)
	{
		elm[i].toTrue();
	}
	int coord_to_index(State3D st)		//test later
	{
		CSpaceConfig* cs = CSpaceConfig::get_instance();
		int dx = (st.x - cs->bottom.x) / cs->range.x;
		int dy = (st.y - cs->bottom.y) / cs->range.y;
		int dth = (st.th - cs->bottom.th) / cs->range.z;
		
		assert((st.x - cs->bottom.x) % cs->range.x == 0);
		assert((st.y - cs->bottom.y) % cs->range.y == 0);
		assert((st.th - cs->bottom.th) % cs->range.z == 0);

		return dx * (cs->numy * cs->numth) + dy * (cs->numth) + dth;
	}

	State3D index_to_coord(int index)	// test later
	{
		CSpaceConfig* cs = CSpaceConfig::get_instance();
		int x = index / (cs->numy * cs->numth);
		int resx = index % (cs->numy * cs->numth);
		int y = resx / (cs->numth);
		int th = resx % (cs->numth);

		return State3D(cs->bottom.x  + x * cs->range.x
					 , cs->bottom.y  + y * cs->range.y
					 , cs->bottom.th + th* cs->range.z);
	}
};
