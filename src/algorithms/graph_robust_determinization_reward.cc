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

#include <cstdlib>
#include <cmath>
#include <limits>
#include <sstream>

#include "graph_robust_determinization_reward.h"
#include "models/base_determinization.h"


// CLASS GraphRobustDeterminizationReward

GraphRobustDeterminizationReward::GraphRobustDeterminizationReward(const Problem& pb, double epsilon, double discount_factor,
		unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth,
		double probability_weight, double reward_weight, unsigned int timeout, unsigned int max_trials)
try : BaseAlgorithm(pb, epsilon, discount_factor), GraphAlgorithm<GraphStateWT<> >(pb, epsilon, discount_factor),
		DeterminizationAlgorithm(pb, epsilon, discount_factor, plan_length, determinization_type, deterministic_planner_type),
		ParticleAlgorithm<GraphStateWT<>*>(pb, epsilon, discount_factor, probabilistic_threshold, nb_of_particles, trajectories_max_depth),
		probability_weight_(probability_weight), reward_weight_(reward_weight), timeout_(timeout), max_trials_(max_trials)
{
    cast_determinization_ = dynamic_cast<BaseDeterminizationWithFluents*>(determinization_);

    if (!cast_determinization_)
        throw BaseException("Determinization type must keep original domain's fluents");

	tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);//GOAL);
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphRobustDeterminizationReward::GraphRobustDeterminizationReward");
	throw;
}


bool GraphRobustDeterminizationReward::policy_defined(const PddlState& st)
{
	return (mdp_->expanded(st));
}


void GraphRobustDeterminizationReward::solve_initialize(const PddlState& st)
{
	initial_state_ = &(mdp_->add_state(st));
	reachable_tip_states_.insert(std::make_pair(initial_state_, 1.0));
}


void GraphRobustDeterminizationReward::solve_progress()
{
	expand_tip_states();
	compute_reachability();
}


bool GraphRobustDeterminizationReward::has_converged()
{
	extern int verbosity; // defined in main.cc

	if (verbosity >= 1)
	{
		std::cerr << "Exit probability: " << exit_probability_ << std::endl;
		std::cerr << (mdp_->get_states().size()) << " explored states" << std::endl;
		std::cerr << reachable_tip_states_.size() << " reachable tip states" << std::endl;
	}

	return (exit_probability_ < probabilistic_threshold_);
}


void GraphRobustDeterminizationReward::compute_reachability()
{
	exit_probability_ = 0.0;
	double nb_particles_inv = 1.0 / static_cast<double>(particles_.size());
	reachable_tip_states_.clear();

	for (particles_list_t::iterator Ip = particles_.begin() ; Ip != particles_.end() ; ++Ip)
	{
		Ip->set_state(initial_state_);
		unsigned int cnt = 0;

		while ((cnt < trajectories_max_depth_) && (!(mdp_->goal(*(Ip->get_state())))))
		{
			if (!(Ip->get_state()->expanded()))
			{
				exit_probability_ += nb_particles_inv;

				std::pair<states_map_t::iterator, bool> res = reachable_tip_states_.insert(std::make_pair(Ip->get_state(), nb_particles_inv));

				if (!res.second)
					res.first->second += nb_particles_inv;

				break;
			}

			Ip->change_state(*mdp_);
			cnt++;
		}
	}
}


void GraphRobustDeterminizationReward::expand_tip_states()
{
	for (states_map_t::iterator Ist = reachable_tip_states_.begin() ; Ist != reachable_tip_states_.end() ; ++Ist)
	{
		if ((Ist->second) >= probabilistic_threshold_)
			generate_trajectory(*(Ist->first));
	}
}


void GraphRobustDeterminizationReward::generate_trajectory(GraphStateWT<>& st)
{
    DeterministicPlanner::plan_t plan;
    find_plan(st, plan);
	GraphStateWT<>* current_st = &st;

	if (plan.empty()) // dead-end state
	{
		expand_state_trajectory(st);
		unsigned int cnt = 0;
		unsigned int picked_action = static_cast<unsigned int>(std::floor(rand()/(RAND_MAX + 1.0) * st.get_transitions().size()));

		for (GraphStateWT<>::actions_map_t::const_iterator Ia = st.get_transitions().begin() ; Ia != st.get_transitions().end() ; ++Ia)
		{
			if (cnt == picked_action)
			{
				st.set_policy(*(Ia->first));
				break;
			}

			cnt++;
		}

		if (discount_factor_ < 1.0d) // assume discount factor set with infinite horizon only
            st.set_value((cast_determinization_->get_reward_min()) / (1.0 - discount_factor_));
        else
            st.set_value((cast_determinization_->get_reward_min()) * trajectories_max_depth_);

		return;
	}

	for (DeterministicPlanner::plan_t::const_iterator Ip = plan.begin() ; Ip != plan.end() ; ++Ip)
	{
		PddlState outcome;
		const Action* action;
		determinization_->effect(*tb_, current_st->get_pddl_state(), **Ip, outcome, action);

		if (!(mdp_->expanded(*current_st)))
        {
            expand_state_trajectory(*current_st);
            current_st->set_policy(*action);
        }

		current_st = &(mdp_->_state_(outcome));
	}

	if (!(mdp_->expanded(*current_st)))
		expand_state_trajectory(*current_st);
}


void GraphRobustDeterminizationReward::expand_state_trajectory(GraphStateWT<>& st)
{
	mdp_->transitions(st);
}


double GraphRobustDeterminizationReward::goal_objective(double log_probability, double reward) const
{
    return ((probability_weight_ * log_probability) + (reward_weight_ * reward));
}




// GraphRobustDeterminizationOptimizedReward

GraphRobustDeterminizationOptimizedReward::GraphRobustDeterminizationOptimizedReward(const Problem& pb, double epsilon, double discount_factor,
		unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth,
		double probability_weight, double reward_weight, unsigned int timeout, unsigned int max_trials)
try : BaseAlgorithm(pb, epsilon, discount_factor), GraphRobustDeterminizationReward(pb, epsilon, discount_factor, plan_length,
                                                                                      determinization_type, deterministic_planner_type,
                                                                                      probabilistic_threshold, nb_of_particles, trajectories_max_depth,
                                                                                      probability_weight, reward_weight, timeout, max_trials)
{

}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphRobustDeterminizationOptimizedReward::GraphRobustDeterminizationOptimizedReward");
	throw;
}


void GraphRobustDeterminizationOptimizedReward::compute_reachability()
{
	exit_probability_ = 0.0;
	double nb_particles_inv = 1.0 / static_cast<double>(particles_.size());
	reachable_tip_states_.clear();
	reachable_states_.clear();

	for (particles_list_t::iterator Ip = particles_.begin() ; Ip != particles_.end() ; ++Ip)
	{
		Ip->set_state(initial_state_);
		reachable_states_.insert(initial_state_);
		unsigned int cnt = 0;

		while ((cnt < trajectories_max_depth_) && (!(mdp_->goal(*(Ip->get_state())))))
		{
			if (!(Ip->get_state()->expanded()))
			{
				exit_probability_ += nb_particles_inv;

				std::pair<states_map_t::iterator, bool> res = reachable_tip_states_.insert(std::make_pair(Ip->get_state(), nb_particles_inv));

				if (!res.second)
					res.first->second += nb_particles_inv;

				break;
			}
			else
            {
                reachable_states_.insert(Ip->get_state());
            }

			Ip->change_state(*mdp_);
			cnt++;
		}
	}
}


void GraphRobustDeterminizationOptimizedReward::solve_progress()
{
    GraphRobustDeterminizationReward::solve_progress();
    optimize(reachable_states_);
}




// GraphRobustDeterminizationRewardOjectiveDichotomy

GraphRobustDeterminizationRewardObjectiveDichotomy::GraphRobustDeterminizationRewardObjectiveDichotomy(const Problem& pb, double epsilon, double discount_factor,
		unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth,
		double probability_weight, double reward_weight, unsigned int timeout, unsigned int max_trials)
try : BaseAlgorithm(pb, epsilon, discount_factor), GraphRobustDeterminizationReward(pb, epsilon, discount_factor, plan_length,
                                                                                      determinization_type, deterministic_planner_type,
                                                                                      probabilistic_threshold, nb_of_particles, trajectories_max_depth,
                                                                                      probability_weight, reward_weight, timeout, max_trials)
{

}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphRobustDeterminizationRewardObjectiveDichotomy::GraphRobustDeterminizationRewardObjectiveDichotomy");
	throw;
}


void GraphRobustDeterminizationRewardObjectiveDichotomy::find_plan(GraphStateWT<>& st, DeterministicPlanner::plan_t& plan)
{
    double min_obj = goal_objective(std::log(cast_determinization_->get_probability_min()) * trajectories_max_depth_, cast_determinization_->get_reward_min() * trajectories_max_depth_);
    double max_obj = goal_objective(std::log(cast_determinization_->get_probability_max()) * trajectories_max_depth_, cast_determinization_->get_reward_max() * trajectories_max_depth_);
    double obj = (min_obj + max_obj) / 2.0;
    unsigned int trial = 0;

    while (((max_obj - min_obj) > epsilon_) && (trial < max_trials_))
    {
        trial++;
        make_problem(st, obj);
        plan.clear();
        deterministic_planner_->solve_with_timeout(plan, timeout_);

        if (!plan.empty())
        {
            min_obj = obj;
            obj = (obj + max_obj) / 2.0;
        }
        else
        {
            max_obj = obj;
            obj = (min_obj + obj) / 2.0;
        }
    }
}


void GraphRobustDeterminizationRewardObjectiveDichotomy::make_problem(GraphStateWT<>& st, double obj)
{
    const Fluent& pfluent = cast_determinization_->get_probability_fluent();
    const Expression& pwexp = *new Value(probability_weight_);
    RCObject::ref(&pwexp);
    const Expression& pmul = Multiplication::make(pwexp, pfluent);
    RCObject::ref(&pmul);
    RCObject::destructive_deref(&pwexp);
    const Fluent& rfluent = cast_determinization_->get_reward_fluent();
    const Expression& rwexp = *new Value(reward_weight_);
    RCObject::ref(&rwexp);
    const Expression& rmul = Multiplication::make(rwexp, rfluent);
    RCObject::ref(&rmul);
    RCObject::destructive_deref(&rwexp);
    const Expression& add = Addition::make(pmul, rmul);
    RCObject::ref(&add);
    RCObject::destructive_deref(&pmul);
    RCObject::destructive_deref(&rmul);
    const Expression& objexp = *new Value(obj);
    RCObject::ref(&objexp);
    const StateFormula& gf = GreaterThanOrEqualTo::make(add, objexp);
    RCObject::ref(&gf);
    RCObject::destructive_deref(&add);
    RCObject::destructive_deref(&objexp);

	determinization_->make_problem(st.get_pddl_state().atoms(), st.get_pddl_state().values(), gf);

	RCObject::destructive_deref(&gf);
}




// GraphRobustDeterminizationOptimizedRewardObjectiveDichotomy

GraphRobustDeterminizationOptimizedRewardObjectiveDichotomy::GraphRobustDeterminizationOptimizedRewardObjectiveDichotomy(const Problem& pb, double epsilon, double discount_factor,
		unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth,
		double probability_weight, double reward_weight, unsigned int timeout, unsigned int max_trials)
try : BaseAlgorithm(pb, epsilon, discount_factor),
      GraphRobustDeterminizationReward(pb, epsilon, discount_factor, plan_length,
                                       determinization_type, deterministic_planner_type,
                                       probabilistic_threshold, nb_of_particles, trajectories_max_depth,
                                       probability_weight, reward_weight, timeout, max_trials),
      GraphRobustDeterminizationOptimizedReward(pb, epsilon, discount_factor, plan_length,
                                                determinization_type, deterministic_planner_type,
                                                probabilistic_threshold, nb_of_particles, trajectories_max_depth,
                                                probability_weight, reward_weight, timeout, max_trials),
      GraphRobustDeterminizationRewardObjectiveDichotomy(pb, epsilon, discount_factor, plan_length,
                                                         determinization_type, deterministic_planner_type,
                                                         probabilistic_threshold, nb_of_particles, trajectories_max_depth,
                                                         probability_weight, reward_weight, timeout, max_trials)
{

}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphRobustDeterminizationOptimizedRewardObjectiveDichotomy::GraphRobustDeterminizationOptimizedRewardObjectiveDichotomy");
	throw;
}




// GraphRobustDeterminizationRewardObjectiveOptimization

GraphRobustDeterminizationRewardObjectiveOptimization::GraphRobustDeterminizationRewardObjectiveOptimization(const Problem& pb, double epsilon, double discount_factor,
		unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth,
		double probability_weight, double reward_weight, unsigned int timeout, unsigned int max_trials)
try : BaseAlgorithm(pb, epsilon, discount_factor), GraphRobustDeterminizationReward(pb, epsilon, discount_factor, plan_length,
                                                                                      determinization_type, deterministic_planner_type,
                                                                                      probabilistic_threshold, nb_of_particles, trajectories_max_depth,
                                                                                      probability_weight, reward_weight, timeout, max_trials)
{

}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphRobustDeterminizationRewardObjectiveOptimization::GraphRobustDeterminizationRewardObjectiveOptimization");
	throw;
}


void GraphRobustDeterminizationRewardObjectiveOptimization::find_plan(GraphStateWT<>& st, DeterministicPlanner::plan_t& plan)
{
    make_problem(st);
    deterministic_planner_->solve_with_timeout(plan, timeout_, true);
}


void GraphRobustDeterminizationRewardObjectiveOptimization::make_problem(GraphStateWT<>& st)
{
    const Fluent& pfluent = cast_determinization_->get_probability_fluent();
    const Expression& pwexp = *new Value(probability_weight_);
    RCObject::ref(&pwexp);
    const Expression& pmul = Multiplication::make(pwexp, pfluent);
    RCObject::ref(&pmul);
    RCObject::destructive_deref(&pwexp);
    const Fluent& rfluent = cast_determinization_->get_reward_fluent();
    const Expression& rwexp = *new Value(reward_weight_);
    RCObject::ref(&rwexp);
    const Expression& rmul = Multiplication::make(rwexp, rfluent);
    RCObject::ref(&rmul);
    RCObject::destructive_deref(&rwexp);
    const Expression& add = Addition::make(pmul, rmul);
    RCObject::ref(&add);
    RCObject::destructive_deref(&pmul);
    RCObject::destructive_deref(&rmul);

	cast_determinization_->make_problem(st.get_pddl_state().atoms(), st.get_pddl_state().values(), add);

	RCObject::destructive_deref(&add);
}




// GraphRobustDeterminizationOptimizedRewardObjectiveOptimization

GraphRobustDeterminizationOptimizedRewardObjectiveOptimization::GraphRobustDeterminizationOptimizedRewardObjectiveOptimization(const Problem& pb, double epsilon, double discount_factor,
		unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth,
		double probability_weight, double reward_weight, unsigned int timeout, unsigned int max_trials)
try : BaseAlgorithm(pb, epsilon, discount_factor),
      GraphRobustDeterminizationReward(pb, epsilon, discount_factor, plan_length,
                                       determinization_type, deterministic_planner_type,
                                       probabilistic_threshold, nb_of_particles, trajectories_max_depth,
                                       probability_weight, reward_weight, timeout, max_trials),
      GraphRobustDeterminizationOptimizedReward(pb, epsilon, discount_factor, plan_length,
                                                determinization_type, deterministic_planner_type,
                                                probabilistic_threshold, nb_of_particles, trajectories_max_depth,
                                                probability_weight, reward_weight, timeout, max_trials),
      GraphRobustDeterminizationRewardObjectiveOptimization(pb, epsilon, discount_factor, plan_length,
                                                            determinization_type, deterministic_planner_type,
                                                            probabilistic_threshold, nb_of_particles, trajectories_max_depth,
                                                            probability_weight, reward_weight, timeout, max_trials)
{

}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphRobustDeterminizationOptimizedRewardObjectiveOptimization::GraphRobustDeterminizationOptimizedRewardObjectiveOptimization");
	throw;
}
