/*
   ppddl-planner - client for IPPC'08

   Copyright (C) 2008 Florent Teichteil-Koenigsbuch and Guillaume Infantes and Ugur Kuter

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

*/

#include <sys/time.h>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <sstream>

#include "base_planner.h"
#include "algorithms/base_algorithm.h"
#include "algorithms/graph_vi.h"
#include "algorithms/symbolic_vi.h"
#include "algorithms/graph_fsp.h"
#include "algorithms/graph_fsp_star.h"
#include "algorithms/graph_trfsp_star.h"
#include "algorithms/graph_bfdt.h"
#include "algorithms/graph_psp_star.h"
#include "algorithms/graph_fdp.h"
#include "algorithms/graph_lao.h"
#include "algorithms/graph_improved_lao.h"
#include "algorithms/graph_rtdp.h"
#include "algorithms/graph_lrtdp.h"
#include "algorithms/graph_ldfs.h"
#include "algorithms/graph_robust_determinization_goal.h"
#include "algorithms/graph_optimal_determinization_goal.h"
#include "algorithms/graph_robust_determinization_reward.h"
#include "algorithms/graph_gco.h"
#include "algorithms/graph_mvpi.h"
#include "algorithms/graph_mvas.h"
#include "algorithms/graph_mvssp.h"
#include "algorithms/symbolic_gco.h"
#include "algorithms/symbolic_fsp.h"
#include "algorithms/symbolic_fsp_star.h"
#include "algorithms/symbolic_lao.h"
#include "algorithms/symbolic_rtdp.h"
#include "commons/base_exceptions.h"


/* Returns the current time in milliseconds. */
long BasePlanner::get_time_milli() {
  struct timeval t;
  gettimeofday(&t, 0);
  return (t.tv_sec%100000)*1000 + t.tv_usec/1000;
}

/* Returns TRUE if RAM is available */
bool BasePlanner::ram_available()
{
	return true;
	// We don't test RAM availability in the competition

	FILE * check_mem_stdout = popen("planner_check_mem.sh ram -q", "r");

	if (!check_mem_stdout)
	{
		std::cerr << "WARNING: unable to check RAM because popen fails in 'BasePlanner::ram_available'" << std::endl;
		return true;
	}

	std::string ramp_str;

	while (true)
	{
		char c = getc(check_mem_stdout);

		if ((c == ':') || (c == EOF))
			break;
		else
			ramp_str += c;
	}

	std::istringstream ramp_ss(ramp_str);
	double ramp;
	ramp_ss >> ramp;
	pclose(check_mem_stdout);
	return (ramp < 95.0);
}


BasePlanner::BasePlanner(const Problem& problem, algorithm_t alg, heuristic_t heur, encoding_t enc,
		determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
		double epsilon, double discount_factor,	double probabilistic_threshold,
		unsigned int nb_of_particles, unsigned int trajectories_max_depth, unsigned int goals_limit,
		double deterministic_probability_weight, double deterministic_reward_weight,
        unsigned int deterministic_planner_timeout, unsigned int deterministic_planner_max_trials,
		double proportion_updated_states, double minimum_mean, double maximum_variance)
: problem_(problem), algorithm_(NULL), algorithm_type_(alg), heuristic_type_(heur), encoding_type_(enc),
	determinization_type_(determinization_type), deterministic_planner_type_(deterministic_planner_type),
	epsilon_(epsilon), discount_factor_(discount_factor), probabilistic_threshold_(probabilistic_threshold),
	nb_of_particles_(nb_of_particles), trajectories_max_depth_(trajectories_max_depth), goals_limit_(goals_limit),
	deterministic_probability_weight_(deterministic_probability_weight), deterministic_reward_weight_(deterministic_reward_weight),
	deterministic_planner_timeout_(deterministic_planner_timeout), deterministic_planner_max_trials_(deterministic_planner_max_trials),
	proportion_updated_states_(proportion_updated_states), minimum_mean_(minimum_mean), maximum_variance_(maximum_variance)
{
	std::srand(std::time(0));
	construct_algorithm();
}


BasePlanner::~BasePlanner()
{
	if (algorithm_)
	{
		delete algorithm_;
		algorithm_ = NULL;
	}
}


void BasePlanner::construct_algorithm()
{
	switch (algorithm_type_)
	{
		case ALGORITHM_VI :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphVI(problem_, epsilon_, discount_factor_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					algorithm_ = new SymbolicVI(problem_, epsilon_, discount_factor_);
					break;
				}
			}

			random_algorithm_ = false;
			break;
		}

		case ALGORITHM_FSP :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphFSP(problem_, epsilon_, discount_factor_, trajectories_max_depth_, heuristic_type_, determinization_type_, deterministic_planner_type_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					algorithm_ = new SymbolicFSP(problem_, epsilon_, discount_factor_, trajectories_max_depth_, heuristic_type_, determinization_type_, deterministic_planner_type_);
					break;
				}
			}

			random_algorithm_ = false;
			break;
		}

		case ALGORITHM_FSP_STAR :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphFSPstar(problem_, epsilon_, discount_factor_, trajectories_max_depth_, heuristic_type_, determinization_type_, deterministic_planner_type_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					algorithm_ = new SymbolicFSPstar(problem_, epsilon_, discount_factor_, trajectories_max_depth_, heuristic_type_, determinization_type_, deterministic_planner_type_);
					break;
				}
			}

			random_algorithm_ = false;
			break;
		}

		case ALGORITHM_TRFSP_STAR :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphTrFSPstar(problem_, epsilon_, discount_factor_, trajectories_max_depth_, heuristic_type_, determinization_type_, deterministic_planner_type_, probabilistic_threshold_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicTrFSPstar(problem, epsilon, discount_factor, heur);
					break;
				}
			}

			random_algorithm_ = false;
			break;
		}

		case ALGORITHM_BFDT :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphBFDT(problem_, epsilon_, discount_factor_, trajectories_max_depth_, heuristic_type_, determinization_type_, deterministic_planner_type_, trajectories_max_depth_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicBFDT(problem_, epsilon_, discount_factor_, heuristic_type_, trajectories_max_depth_);
					break;
				}
			}

			random_algorithm_ = false;
			break;
		}

		case ALGORITHM_PSP_STAR :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphPSPstar(problem_, epsilon_, discount_factor_, heuristic_type_, determinization_type_, deterministic_planner_type_, probabilistic_threshold_, nb_of_particles_, trajectories_max_depth_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicPSPstar(problem, epsilon, discount_factor, heur);
					break;
				}
			}

			random_algorithm_ = true;
			break;
		}

		case ALGORITHM_FDP :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphFDP(problem_, epsilon_, discount_factor_, trajectories_max_depth_, heuristic_type_, determinization_type_, deterministic_planner_type_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicFDP(problem, epsilon, discount_factor, heur);
					break;
				}
			}

			random_algorithm_ = false;
			break;
		}

		case ALGORITHM_LAO :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphLAO(problem_, epsilon_, discount_factor_, trajectories_max_depth_, heuristic_type_, determinization_type_, deterministic_planner_type_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					algorithm_ = new SymbolicLAO(problem_, epsilon_, discount_factor_, trajectories_max_depth_, heuristic_type_, determinization_type_, deterministic_planner_type_);
					break;
				}
			}

			random_algorithm_ = false;
			break;
		}

		case ALGORITHM_IMPROVED_LAO :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphImprovedLAO(problem_, epsilon_, discount_factor_, trajectories_max_depth_, heuristic_type_, determinization_type_, deterministic_planner_type_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicLAO(problem, epsilon, discount_factor, heur);
					break;
				}
			}

			random_algorithm_ = false;
			break;
		}

		case ALGORITHM_RTDP :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphRTDP(problem_, epsilon_, discount_factor_, heuristic_type_, determinization_type_, deterministic_planner_type_, trajectories_max_depth_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					algorithm_ = new SymbolicRTDP(problem_, epsilon_, discount_factor_, heuristic_type_, determinization_type_, deterministic_planner_type_, trajectories_max_depth_);
					break;
				}
			}

			random_algorithm_ = true;
			break;
		}

		case ALGORITHM_LRTDP :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphLRTDP(problem_, epsilon_, discount_factor_, heuristic_type_, determinization_type_, deterministic_planner_type_, trajectories_max_depth_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicLRTDP(problem_, epsilon_, discount_factor_, heuristic_type_, trajectories_max_depth_);
					break;
				}
			}

			random_algorithm_ = true;
			break;
		}

		case ALGORITHM_LDFS :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphLDFS(problem_, epsilon_, discount_factor_, trajectories_max_depth_, heuristic_type_, determinization_type_, deterministic_planner_type_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicLDFS(problem_, epsilon_, discount_factor_, heuristic_type_);
					break;
				}
			}

			random_algorithm_ = false;
			break;
		}

		case ALGORITHM_OD :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphOptimalDeterminizationGoal(problem_, epsilon_, discount_factor_, heuristic_type_, determinization_type_, deterministic_planner_type_, probabilistic_threshold_, nb_of_particles_, trajectories_max_depth_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicRobustFF(problem_, epsilon_, discount_factor_, heuristic_type_, trajectories_max_depth_);
					break;
				}
			}

			random_algorithm_ = false;
			break;
		}

		case ALGORITHM_RDPG :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphRobustDeterminizationProblemGoals(problem_, epsilon_, discount_factor_, trajectories_max_depth_, determinization_type_, deterministic_planner_type_, probabilistic_threshold_, nb_of_particles_, trajectories_max_depth_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicRobustFF(problem_, epsilon_, discount_factor_, heuristic_type_, trajectories_max_depth_);
					break;
				}
			}

			random_algorithm_ = true;
			break;
		}

		case ALGORITHM_RDRG :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphRobustDeterminizationRandomGoals(problem_, epsilon_, discount_factor_, trajectories_max_depth_, determinization_type_, deterministic_planner_type_, probabilistic_threshold_, nb_of_particles_, trajectories_max_depth_, goals_limit_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicRobustFF(problem_, epsilon_, discount_factor_, heuristic_type_, trajectories_max_depth_);
					break;
				}
			}

			random_algorithm_ = true;
			break;
		}

		case ALGORITHM_RDBG :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphRobustDeterminizationBestGoals(problem_, epsilon_, discount_factor_, trajectories_max_depth_, determinization_type_, deterministic_planner_type_, probabilistic_threshold_, nb_of_particles_, trajectories_max_depth_, goals_limit_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicRobustFF(problem_, epsilon_, discount_factor_, heuristic_type_, trajectories_max_depth_);
					break;
				}
			}

			random_algorithm_ = true;
			break;
		}

		case ALGORITHM_RDRD :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphRobustDeterminizationRewardObjectiveDichotomy(problem_, epsilon_, discount_factor_, trajectories_max_depth_, determinization_type_, deterministic_planner_type_, probabilistic_threshold_, nb_of_particles_, trajectories_max_depth_, deterministic_probability_weight_, deterministic_reward_weight_, deterministic_planner_timeout_, deterministic_planner_max_trials_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicRobustFF(problem_, epsilon_, discount_factor_, heuristic_type_, trajectories_max_depth_);
					break;
				}
			}

			random_algorithm_ = true;
			break;
		}

		case ALGORITHM_RDRO :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphRobustDeterminizationRewardObjectiveOptimization(problem_, epsilon_, discount_factor_, trajectories_max_depth_, determinization_type_, deterministic_planner_type_, probabilistic_threshold_, nb_of_particles_, trajectories_max_depth_, deterministic_probability_weight_, deterministic_reward_weight_, deterministic_planner_timeout_, deterministic_planner_max_trials_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicRobustFF(problem_, epsilon_, discount_factor_, heuristic_type_, trajectories_max_depth_);
					break;
				}
			}

			random_algorithm_ = true;
			break;
		}

		case ALGORITHM_RDRD_STAR :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphRobustDeterminizationOptimizedRewardObjectiveDichotomy(problem_, epsilon_, discount_factor_, trajectories_max_depth_, determinization_type_, deterministic_planner_type_, probabilistic_threshold_, nb_of_particles_, trajectories_max_depth_, deterministic_probability_weight_, deterministic_reward_weight_, deterministic_planner_timeout_, deterministic_planner_max_trials_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicRobustFF(problem_, epsilon_, discount_factor_, heuristic_type_, trajectories_max_depth_);
					break;
				}
			}

			random_algorithm_ = true;
			break;
		}

		case ALGORITHM_RDRO_STAR :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphRobustDeterminizationOptimizedRewardObjectiveOptimization(problem_, epsilon_, discount_factor_, trajectories_max_depth_, determinization_type_, deterministic_planner_type_, probabilistic_threshold_, nb_of_particles_, trajectories_max_depth_, deterministic_probability_weight_, deterministic_reward_weight_, deterministic_planner_timeout_, deterministic_planner_max_trials_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicRobustFF(problem_, epsilon_, discount_factor_, heuristic_type_, trajectories_max_depth_);
					break;
				}
			}

			random_algorithm_ = true;
			break;
		}

        case ALGORITHM_GCO :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphGCO(problem_, epsilon_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					algorithm_ = new SymbolicGCO(problem_, epsilon_);
					break;
				}
			}

			random_algorithm_ = false;
			break;
		}

		case ALGORITHM_MVPFPI :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphMVPFPI(problem_, epsilon_, discount_factor_, proportion_updated_states_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicMVPFPI(problem_, epsilon_, discount_factor_, proportion_updated_states_);
					break;
				}
			}

			random_algorithm_ = false;
			break;
		}

		case ALGORITHM_MVHOPI :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphMVHOPI(problem_, minimum_mean_, maximum_variance_, epsilon_, discount_factor_, proportion_updated_states_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicMVHOPI(problem_, minimum_mean_, maximum_variance_, epsilon_, discount_factor_, proportion_updated_states_);
					break;
				}
			}

			random_algorithm_ = false;
			break;
		}

		case ALGORITHM_MVPFAS :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphMVPFAS(problem_, epsilon_, discount_factor_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicMVPFAS(problem_, epsilon_, discount_factor_);
					break;
				}
			}

			random_algorithm_ = true;
			break;
		}

		case ALGORITHM_MVHOAS :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphMVHOAS(problem_, minimum_mean_, maximum_variance_, epsilon_, discount_factor_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicMVHOAS(problem_, minimum_mean_, maximum_variance_, epsilon_, discount_factor_);
					break;
				}
			}

			random_algorithm_ = true;
			break;
		}

		case ALGORITHM_MVPFSSP :
		{
			switch (encoding_type_)
			{
				case ENCODING_GRAPH :
				{
					algorithm_ = new GraphMVPFSSP(problem_, epsilon_, proportion_updated_states_);
					break;
				}

				case ENCODING_SYMBOLIC :
				{
					//algorithm_ = new SymbolicMVPFSSP(problem_, epsilon_, proportion_updated_states_);
					break;
				}
			}

			random_algorithm_ = false;
			break;
		}

        default:
            throw BaseException("unhandled algorithm", "BasePlanner::construct_algorithm");
	}
}
