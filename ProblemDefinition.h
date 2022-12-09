#pragma once

#include "Node.h"
#include "GoalJudge.h"

class ProblemDefinition
{
public:
	Node initial;
	GoalJudge gj;

public:
	ProblemDefinition();

//	bool goal_judge(PointCloud pc);
//	bool semigoal_judge(PointCloud pc);
	int judge_once(PointCloud pc);
	bool judge_upper(std::vector<PointCloud> pcs)
	{
		return gj.judge_upper(pcs);
	}
};

Node read_initial();