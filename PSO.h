
#include <climits>

#include "icmMath.h"
#include "CFreeICS.h"
#include "Controller.h"
#include "Problem.h"
#include "RRT.h"
#include "TaskSet.h"


double caging_func(Node node);


class PSO
{
private:
	std::vector<Node> particles;
	std::vector<Node> velocity;
	std::vector<Node> personal_best;
	std::vector<double> personal_score;
	Node global_best;
	double global_score;

	int repeat_times, particle_nums;

public:
	PSO(int _repeat_times, int _particle_nums)
		:repeat_times(_repeat_times), 
	     particle_nums(_particle_nums)
	{
	}

	PSO()
		:repeat_times(200),
		 particle_nums(5)
	{}

	std::vector<double> optimize(Node ini)
	{
		init(ini);

		for(int i=0; i<repeat_times; ++i)
		{
			std::cout << i << ": " << std::endl;
			update_personal();
			update_global();
		}

		std::cout << global_best << std::endl;
		return std::vector<double>();
	}


	void init(Node ini)
	{
		std::srand(time(NULL));

		const double diffusion_width = 10;
		for(int i=0; i<particle_nums; ++i){
			std::vector<double> tmpnode;
			for(int n=0; n<Node::dof; ++n){
				double random = (double)rand()/RAND_MAX;
				double tmpangle = ini.get_element(n) + diffusion_width*random - (diffusion_width/2);
				if(tmpangle>90)		tmpangle=90;
				if(tmpangle<-90)	tmpangle=-90;
				tmpnode.push_back(tmpangle);
			}
			assert(tmpnode.size() == 6);
			particles.push_back(Node(tmpnode));
		}
	
		velocity.resize(particle_nums);

		personal_best = particles;
		for(int i=0; i<personal_best.size(); ++i){
			double tmpscore = caging_func(personal_best[i]);
			personal_score.push_back(tmpscore);
			std::cout << personal_best[i] << " : " << tmpscore << std::endl;
		}
	
		double tmpmin = DBL_MAX;	int global_index = 0;
		for(int i=0; i<personal_score.size(); ++i){
			if(tmpmin > personal_score[i]){
				tmpmin = personal_score[i];
				global_index = i;
			}
		}
		global_best = personal_best[global_index];
		global_score = tmpmin;

		std::cout << global_best << " : " << global_score << std::endl;
	}



	void update_personal()
	{
		const double w=0.5, c1=0.5, c2 = 1.5;

		for(int i=0; i<particles.size(); ++i){
			double r1 = (double)rand()/RAND_MAX;
			double r2 = (double)rand()/RAND_MAX;
			velocity[i] = velocity[i] * w +
				r1 * c1 * (personal_best[i] - particles[i]) + 
				r2 * c2 * (global_best - particles[i]);
			particles[i] = particles[i] + velocity[i];
			for(int n=0; n<Node::dof; ++n){
				if(particles[i][n]>90)	particles[i][n] = 90;
				if(particles[i][n]<-90)	particles[i][n] = -90;
			}

			double tmpscore = caging_func(particles[i]);
			if(tmpscore < personal_score[i]){
				personal_score[i] = tmpscore;
				personal_best[i] = particles[i];
			}
		}
	}


	void update_global()
	{
		for(int i=0; i<personal_best.size(); ++i){
			if(personal_score[i] < global_score){
				global_score = personal_score[i];
				global_best = personal_best[i];
			}
		}
		std::cout << global_best << " : " << global_score << std::endl;
	}

};

