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
   GNU General Public Lixcense for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

*/

#include "rddl_planner.h"
#include "algorithms/base_algorithm.h"
#include "commons/base_exceptions.h"

RddlPlanner::RddlPlanner(const Problem& problem, algorithm_t alg, heuristic_t heur, encoding_t enc,
		determinization_t determinization_type, deterministic_planner_t deterministic_planner_type, double epsilon, double discount_factor,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth, unsigned int goals_limit,
		double deterministic_probability_weight, double deterministic_reward_weight,
        unsigned int deterministic_planner_timeout, unsigned int deterministic_planner_max_trials,
		double proportion_updated_states, double minimum_mean, double maximum_variance)
: BasePlanner(problem, alg, heur, enc, determinization_type, deterministic_planner_type, epsilon, discount_factor, probabilistic_threshold,
				nb_of_particles, trajectories_max_depth, goals_limit, deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance), Planner(problem)
{
}


void RddlPlanner::initRound( )
{
    time_threshold_ = std::max((long) 1000, static_cast<long>(std::floor(0.05 * 1)));
    round_time_ = get_time_milli();
}

void RddlPlanner::initRound(  int numberOfRounds,  long remainingTime)
{
    time_threshold_ = std::max((long) 1000, static_cast<long>(std::floor(0.05 * remainingTime)));
    round_time_ = get_time_milli();
}

void RddlPlanner::endRound(double const& roundReward)
{
    std::cout << "***********************************************" << std::endl
              << ">>> END OF ROUND " /* << (currentRound + 1) */
              << " -- REWARD RECEIVED: " << roundReward << std::endl
              << "***********************************************\n" << std::endl;
}

const Action* RddlPlanner::decideAction(const AtomSet& atoms, const ValueMap& values)
{
    return decideAction(atoms, values, 0);
}


const Action* RddlPlanner::decideAction(const AtomSet& atoms, const ValueMap& values, long remainingTime)
{
	try
	{
		PddlState state(atoms, values);

		extern int verbosity; // defined in main_planner[test|client].cc

		if (algorithm_->policy_defined(state))
		{
			const Action& action = algorithm_->get_mdp().get_policy(state);

			if ((&action) == (&(algorithm_->get_transitions_blackbox().get_planner_stay_action())))
				return NULL;
			else
				return &action;
		}
		else
		{
			//long current_action_time = get_time_milli();
			algorithm_->solve_initialize(state);

			while((remainingTime - (get_time_milli() - round_time_)) > time_threshold_)
			{
				if (!ram_available())
					break;

				algorithm_->solve_progress();

				if (algorithm_->has_converged())
					break;

			}

			if (! (algorithm_->has_converged()))
				std::cout << "Time out..." << std::endl;

			if (verbosity >= 2)
			{
				std::cout << (algorithm_->get_mdp().print_model()) << std::endl;
				std::cout << (algorithm_->get_mdp().print_solution()) << std::endl;
			}

			if (algorithm_->policy_defined(state))
			{
				const Action& action = algorithm_->get_mdp().get_policy(state);

				if ((&action) == (&(algorithm_->get_transitions_blackbox().get_planner_stay_action())))
					return NULL;
				else
					return &action;
			}
			else
				return randomAction(atoms, values);
		}
	}
	catch (const std::exception& error)
	{
		std::cerr << error.what() << std::endl;
		return randomAction(atoms, values);
	}
}


const Action* RddlPlanner::randomAction(const AtomSet& atoms, const ValueMap& values)
{
	ActionList actions;
	_problem.enabled_actions(actions, atoms, values);

	if (actions.empty())
		return 0;
	else
	{
		size_t i = size_t(rand()/(RAND_MAX + 1.0)*actions.size());
		return actions[i];
	}
}
