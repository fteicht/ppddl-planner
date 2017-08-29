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

#include "test_planner.h"
#include "algorithms/base_algorithm.h"
#include "algorithms/graph_gco.h"

// CLASS TestPlanner

SimulationTestPlanner::SimulationTestPlanner(const Problem& problem, algorithm_t alg, heuristic_t heur, encoding_t enc,
		determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
		double epsilon, double discount_factor, long time_threshold, unsigned int trials_number, unsigned int trials_max_depth,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth, unsigned int goals_limit,
		double deterministic_probability_weight, double deterministic_reward_weight,
        unsigned int deterministic_planner_timeout, unsigned int deterministic_planner_max_trials,
		double proportion_updated_states, double minimum_mean, double maximum_variance,
		bool compute_goal_statistics, algorithm_t goal_statistics_algorithm)
: BasePlanner(problem, alg, heur, enc, determinization_type, deterministic_planner_type, epsilon, discount_factor,
	probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
	deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
	proportion_updated_states, minimum_mean, maximum_variance),
	problem_(problem), current_state_(NULL), initial_time_(0),
	trials_number_(trials_number), trials_max_depth_(trials_max_depth), compute_goal_statistics_(compute_goal_statistics), goal_statistics_algorithm_(goal_statistics_algorithm)
{
	time_threshold_ = time_threshold;
}


SimulationTestPlanner::~SimulationTestPlanner()
{
	if (current_state_)
		delete current_state_;
}


const Action& SimulationTestPlanner::action(const State& st)
{
	try
	{
		PddlState pddl_st(st.atoms(), st.values());
		check_solve(pddl_st);
		return (algorithm_->get_mdp().get_policy(pddl_st));
	}
	catch (TimeoutException& error)
	{
		error.push_function_backtrace("TestPlanner::action");
		throw;
	}
	catch (const BaseException& error)
	{
		throw;
	}
}


double SimulationTestPlanner::value(const State& st)
{
	try
	{
		PddlState pddl_st(st.atoms(), st.values());
		check_solve(pddl_st);
		return (algorithm_->get_mdp().get_value(pddl_st));
	}
	catch (TimeoutException& error)
	{
		error.push_function_backtrace("TestPlanner::value");
		throw;
	}
	catch (const BaseException& error)
	{
		throw;
	}
}


SimulationTestPlanner::SimulationInfo SimulationTestPlanner::launch()
{
	SimulationInfo res;
	res.average_cumulated_value_ = 0.0;
	res.goal_reached_percentage_ = 0.0;
	res.average_turns_number_ = 0.0;
	res.average_time_per_trial_ = 0.0;
	res.nb_of_replannings_ = 0;
	res.average_planning_time_ = 0.0;
	res.time_out_percentage_ = 0.0;
	long trial_resolving_time = 0;
	initial_time_ = get_time_milli();
	solving_time_ = 0;
	extern int verbosity; // defined in main_plannertest.cc

	for (unsigned int trial_cnt = 0 ; trial_cnt < trials_number_ ; trial_cnt++)
	{
		try
		{
			if (current_state_)
				delete current_state_;

			current_state_ = new State(problem_);
			unsigned int step;
			nb_of_replannings_ = 0;
			double average_planning_time = 0.0;

//			if (random_algorithm_ || (test_type_ == TEST_PLANNER_ANYTIME))
//			{
//				if (trial_cnt == 0)
//					trial_resolving_time = solving_time_;
//				else
//				{
//					delete algorithm_;
//					construct_algorithm();
//					resolve(*current_state_);
//					trial_resolving_time = resolving_time_;
//				}
//			}

			for (step = 0 ; step < trials_max_depth_ ; step++)
			{
				if (current_state_->goal())
					break;

				resolving_time_ = 0;
				const State* next_state = &(current_state_->next(action(*current_state_)));
				trial_resolving_time += resolving_time_;
				double nor = res.nb_of_replannings_ + nb_of_replannings_;
				res.average_planning_time_ = (((nor - 1) / nor) * res.average_planning_time_) + (resolving_time_ / nor);
				average_planning_time = ((((double) (nb_of_replannings_ - 1)) / ((double) nb_of_replannings_)) * average_planning_time) + (resolving_time_ / ((double) nb_of_replannings_));
				delete current_state_;
				current_state_ = next_state;
			}

			double coeff = static_cast<double>(trial_cnt) / static_cast<double>(trial_cnt + 1);
			double new_term = problem_.metric().value(current_state_->values()).double_value() / static_cast<double>(trial_cnt + 1);
			res.average_cumulated_value_ = (res.average_cumulated_value_ * coeff) + new_term;

			new_term = static_cast<double>(step) / static_cast<double>(trial_cnt + 1);
			res.average_turns_number_ = (res.average_turns_number_ * coeff) + new_term;

			//if (random_algorithm_)
				new_term = static_cast<double>(trial_resolving_time) / static_cast<double>(trial_cnt + 1);
			//else
				//new_term = static_cast<double>(solving_time_) / static_cast<double>(trial_cnt + 1);

			res.average_time_per_trial_ = (res.average_time_per_trial_ * coeff) + new_term;

			if (current_state_->goal())
				res.goal_reached_percentage_ += (1.0 / static_cast<double>(trials_number_)) * 100.0;

			res.nb_of_replannings_ += nb_of_replannings_;

			if (verbosity >= 1)
			{
				std::cerr << "TRIAL " << trial_cnt << ":" << std::endl;
				std::cerr << "   solving time: " << trial_resolving_time << std::endl;
				std::cerr << "   value: " << problem_.metric().value(current_state_->values()).double_value() << std::endl;
				std::cerr << "   goal reached: " << ((current_state_->goal()) ? ("yes") : ("no")) << std::endl;
				std::cerr << "   turns: " << step << std::endl;
				std::cerr << "   (re)plannings: " << nb_of_replannings_ << std::endl;
				std::cerr << "   average planning time: "  << average_planning_time << std::endl;
				std::cerr << "   reachable state space size: " << (algorithm_->reachable_state_space_size()) << std::endl;
			}
		}
		catch (const TimeoutException& error)
		{
			res.time_out_percentage_ += (1.0 / static_cast<double>(trials_number_)) * 100.0;

			if (verbosity >= 1)
				std::cerr << "TRIAL " << trial_cnt << ": time out!" << std::endl;

			break;
		}
	}

	return res;
}


// CLASS CompleteTestPlanner

SimulationTestCompletePlanner::SimulationTestCompletePlanner(const Problem& problem, algorithm_t alg, heuristic_t heur, encoding_t enc,
		determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
		double epsilon, double discount_factor, long time_threshold,
		unsigned int trials_number, unsigned int trials_max_depth,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth, unsigned int goals_limit,
		double deterministic_probability_weight, double deterministic_reward_weight,
        unsigned int deterministic_planner_timeout, unsigned int deterministic_planner_max_trials,
		double proportion_updated_states, double minimum_mean, double maximum_variance,
		bool compute_goal_statistics, algorithm_t goal_statistics_algorithm)
: SimulationTestPlanner(problem, alg, heur, enc, determinization_type, deterministic_planner_type, epsilon, discount_factor, time_threshold,
		trials_number, trials_max_depth, probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
		deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
		proportion_updated_states, minimum_mean, maximum_variance,
		compute_goal_statistics, goal_statistics_algorithm)
{
}


void SimulationTestCompletePlanner::check_solve(const PddlState& st)
{
	if (!(algorithm_->policy_defined(st)))
	{
		extern int verbosity; // defined in main_plannertest.cc
		long resolving_time_begin = get_time_milli();
		nb_of_replannings_++;

		if (verbosity >= 1)
			std::cerr << "Solving from state " << st << std::endl;

		algorithm_->solve_initialize(st);
		bool alg_converged = false;

		while ((get_time_milli() - initial_time_) <= time_threshold_)
		{
			if (!ram_available())
				break;

			algorithm_->solve_progress();

			if (algorithm_->has_converged())
			{
				alg_converged = true;
				break;
			}
		}

		if (!alg_converged)
			throw TimeoutException("algorithm has not yet converged", "CompleteTestPlanner::check_solve");

		resolving_time_ = (get_time_milli() - resolving_time_begin);
		solving_time_ += resolving_time_;

        if (compute_goal_statistics_)
        {
            std::cout << "Computing goal statistics..." << std::endl;
            std::pair<double, double> res = algorithm_->compute_goal_statistics(goal_statistics_algorithm_, st);
            std::cout << "   goal probability: " << res.first << std::endl;
            std::cout << "   goal cost: " << res.second << std::endl;
        }

		if (verbosity >= 2)
		{
			std::cout << (algorithm_->get_mdp().print_model()) << std::endl;
			std::cout << (algorithm_->get_mdp().print_solution()) << std::endl;
		}
	}
}


// CLASS AnytimeTestPlanner

SimulationTestAnytimePlanner::SimulationTestAnytimePlanner(const Problem& problem, algorithm_t alg, heuristic_t heur, encoding_t enc,
		determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
		double epsilon, double discount_factor, long time_threshold,
		unsigned int trials_number, unsigned int trials_max_depth,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth, unsigned int goals_limit,
		double deterministic_probability_weight, double deterministic_reward_weight,
        unsigned int deterministic_planner_timeout, unsigned int deterministic_planner_max_trials,
		double proportion_updated_states, double minimum_mean, double maximum_variance,
		bool compute_goal_statistics, algorithm_t goal_statistics_algorithm)
: SimulationTestPlanner(problem, alg, heur, enc, determinization_type, deterministic_planner_type, epsilon, discount_factor, time_threshold,
		trials_number, trials_max_depth, probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
		deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
		proportion_updated_states, minimum_mean, maximum_variance,
		compute_goal_statistics, goal_statistics_algorithm)
{
}


void SimulationTestAnytimePlanner::check_solve(const PddlState& st)
{
	long resolving_time_begin = get_time_milli();

	if (algorithm_->policy_defined(st))
	{
		if ((get_time_milli() - initial_time_) <= time_threshold_)
		{
			if (algorithm_->has_converged())
				algorithm_->solve_initialize(st);

			if (ram_available())
				algorithm_->solve_progress();
		}
	}
	else
	{
		if ((get_time_milli() - initial_time_) <= time_threshold_)
		{
			nb_of_replannings_++;
			algorithm_->solve_initialize(st);

			do
			{
				if (!ram_available())
					break;

				algorithm_->solve_progress();
			}
			while (!(algorithm_->policy_defined(st)));
		}
		else
			throw TimeoutException("algorithm has not yet converged", "AnytimeTestPlanner::check_solve");
	}

	resolving_time_ = (get_time_milli() - resolving_time_begin);
	solving_time_ += resolving_time_;
}
