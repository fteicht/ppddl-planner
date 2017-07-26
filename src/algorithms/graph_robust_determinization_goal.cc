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

#include "graph_robust_determinization_goal.h"
#include "models/base_determinization.h"
#include "planners/deterministic_planner.h"


// CLASS GraphRobustDeterminizationGoal

GraphRobustDeterminizationGoal::GraphRobustDeterminizationGoal(const Problem& pb, double epsilon, double discount_factor,
		unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth)
try : BaseAlgorithm(pb, epsilon, discount_factor), GraphAlgorithm<GraphStateWT<> >(pb, epsilon, discount_factor),
		DeterminizationAlgorithm(pb, epsilon, discount_factor, plan_length, determinization_type, deterministic_planner_type),
		ParticleAlgorithm<GraphStateWT<>*>(pb, epsilon, discount_factor, probabilistic_threshold, nb_of_particles, trajectories_max_depth)
{
	tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);//GOAL);
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphRobustDeterminizationGoal::GraphRobustDeterminizationGoal");
	throw;
}


bool GraphRobustDeterminizationGoal::policy_defined(const PddlState& st)
{
	return (mdp_->expanded(st));
}


void GraphRobustDeterminizationGoal::solve_initialize(const PddlState& st)
{
	initial_state_ = &(mdp_->add_state(st));
	reachable_tip_states_.insert(std::make_pair(initial_state_, 1.0));
}


void GraphRobustDeterminizationGoal::solve_progress()
{
	expand_tip_states();
	compute_reachability();
}


bool GraphRobustDeterminizationGoal::has_converged()
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


void GraphRobustDeterminizationGoal::compute_reachability()
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


void GraphRobustDeterminizationGoal::expand_tip_states()
{
	for (states_map_t::iterator Ist = reachable_tip_states_.begin() ; Ist != reachable_tip_states_.end() ; ++Ist)
	{
		if ((Ist->second) >= probabilistic_threshold_)
			generate_trajectory(*(Ist->first));
	}
}


void GraphRobustDeterminizationGoal::generate_trajectory(GraphStateWT<>& st)
{
	make_problem(st);
	DeterministicPlanner::plan_t plan;
	deterministic_planner_->solve(plan);
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

		st.set_value((discount_factor_ < 1.0)?(-1.0 / (1.0 - discount_factor_)):(-std::numeric_limits<double>::max())); // added after the competition
		return;
	}

	for (DeterministicPlanner::plan_t::const_iterator Ip = plan.begin() ; Ip != plan.end() ; ++Ip)
	{
		if (mdp_->expanded(*current_st))
			break;

		expand_state_trajectory(*current_st);
		PddlState outcome;
		const Action* action;
		determinization_->effect(*tb_, current_st->get_pddl_state(), **Ip, outcome, action);
		current_st->set_policy(*action);
		current_st = &(mdp_->_state_(outcome));
	}

	// ADDED AFTER THE COMPETITION //
	if (!(mdp_->expanded(*current_st)))
		expand_state_trajectory(*current_st);
	// ADDED AFTER THE COMPETITION //
}




// CLASS GraphRobustDeterminizationProblemGoals

GraphRobustDeterminizationProblemGoals::GraphRobustDeterminizationProblemGoals(const Problem& pb, double epsilon, double discount_factor,
		unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth)
try : BaseAlgorithm(pb, epsilon, discount_factor), GraphRobustDeterminizationGoal(pb, epsilon, discount_factor, plan_length,
			determinization_type, deterministic_planner_type, probabilistic_threshold, nb_of_particles, trajectories_max_depth)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphRobustDeterminizationProblemGoals::GraphRobustDeterminizationProblemGoals");
	throw;
}


void GraphRobustDeterminizationProblemGoals::make_problem(GraphStateWT<>& st)
{
	determinization_->make_problem(st.get_pddl_state().atoms(), st.get_pddl_state().values(), tb_->get_problem().goal());
}


void GraphRobustDeterminizationProblemGoals::expand_state_trajectory(GraphStateWT<>& st)
{
	mdp_->transitions(st);
}




// CLASS GraphRobustDeterminizationRandomGoals

GraphRobustDeterminizationRandomGoals::GraphRobustDeterminizationRandomGoals(const Problem& pb, double epsilon, double discount_factor,
		unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth, unsigned int goals_limit)
try : BaseAlgorithm(pb, epsilon, discount_factor), GraphRobustDeterminizationGoal(pb, epsilon, discount_factor, plan_length,
			determinization_type, deterministic_planner_type, probabilistic_threshold, nb_of_particles, trajectories_max_depth), goals_limit_(goals_limit)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphRobustDeterminizationRandomGoals::GraphRobustDeterminizationRandomGoals");
	throw;
}


void GraphRobustDeterminizationRandomGoals::make_problem(GraphStateWT<>& st)
{
	unsigned int expanded_states_size = expanded_states_.size();
	std::set<GraphStateWT<>*> chosen_states;

	if (expanded_states_size <= goals_limit_)
	{
		for (states_set_t::const_iterator Ist = expanded_states_.begin() ; Ist != expanded_states_.end() ; ++Ist)
			chosen_states.insert(*Ist);
	}
	else
	{
		std::list<unsigned int> picked_states;

		for (unsigned int cnt = 0 ; cnt < goals_limit_ ; cnt++)
		{
			unsigned int pick = static_cast<unsigned int>(std::floor(static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX) * (expanded_states_size - cnt)));

			if (pick == (expanded_states_size - cnt))
				pick--;

			std::list<unsigned int>::iterator Il;

			for (Il = picked_states.begin() ; Il != picked_states.end() ; ++Il)
			{
				if (pick < (*Il))
					break;
				else
					pick++;
			}

			picked_states.insert(Il, pick);
		}

		unsigned int state_cnt = 0;

		for (states_set_t::iterator Ist = expanded_states_.begin() ; Ist != expanded_states_.end() ; ++Ist)
		{
			if (state_cnt == picked_states.front())
			{
				chosen_states.insert(*Ist);
				picked_states.pop_front();
			}

			state_cnt++;
		}
	}

	const StateFormula* deterministic_goal = &((mdp_->state_formula(chosen_states)) || (tb_->get_problem().goal()));
	RCObject::ref(deterministic_goal);
	determinization_->make_problem(st.get_pddl_state().atoms(), st.get_pddl_state().values(), *deterministic_goal);
	RCObject::destructive_deref(deterministic_goal);
}


void GraphRobustDeterminizationRandomGoals::expand_state_trajectory(GraphStateWT<>& st)
{
	mdp_->transitions(st);
	expanded_states_.insert(&st);
}




// CLASS GraphRobustDeterminizationBestGoals

GraphRobustDeterminizationBestGoals::GraphRobustDeterminizationBestGoals(const Problem& pb, double epsilon, double discount_factor,
		unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth, unsigned int goals_limit)
try : BaseAlgorithm(pb, epsilon, discount_factor), GraphRobustDeterminizationGoal(pb, epsilon, discount_factor, plan_length,
			determinization_type, deterministic_planner_type, probabilistic_threshold, nb_of_particles, trajectories_max_depth), goals_limit_(goals_limit)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphRobustDeterminizationBestGoals::GraphRobustDeterminizationBestGoals");
	throw;
}


void GraphRobustDeterminizationBestGoals::make_problem(GraphStateWT<>& st)
{
	std::set<GraphStateWT<>*> chosen_states;

	if (expanded_states_.size() <= goals_limit_)
	{
		for (states_set_t::const_iterator Ist = expanded_states_.begin() ; Ist != expanded_states_.end() ; ++Ist)
			chosen_states.insert(*Ist);
	}
	else
	{
		optimize(expanded_states_);
		std::multimap<double, GraphStateWT<>*> ranked_states;

		for (states_set_t::const_iterator Ist = expanded_states_.begin() ; Ist != expanded_states_.end() ; ++Ist)
			ranked_states.insert(std::make_pair((*Ist)->get_value(), *Ist));

		std::multimap<double, GraphStateWT<>*>::const_iterator Ist = ranked_states.begin();

		for (unsigned int cnt = 0 ; cnt < goals_limit_ ; cnt++)
		{
			chosen_states.insert(Ist->second);
			++Ist;
		}
	}

	const StateFormula* deterministic_goal = &((mdp_->state_formula(chosen_states)) || (tb_->get_problem().goal()));
	RCObject::ref(deterministic_goal);
	determinization_->make_problem(st.get_pddl_state().atoms(), st.get_pddl_state().values(), *deterministic_goal);
	RCObject::destructive_deref(deterministic_goal);
}


void GraphRobustDeterminizationBestGoals::expand_state_trajectory(GraphStateWT<>& st)
{
	const GraphStateWT<>::actions_map_t* transitions = mdp_->transitions(st);
	expanded_states_.insert(&st);

	for (GraphStateWT<>::actions_map_t::const_iterator Ia = transitions->begin() ; Ia != transitions->end() ; ++Ia)
	{
		for (GraphMDP<GraphStateWT<> >::transition_list_t::const_iterator Ieff = Ia->second.begin() ; Ieff != Ia->second.end() ; ++Ieff)
		{
			if (expanded_states_.find(&((*Ieff)->_outcome_())) == expanded_states_.end()) // added after the competition
				(*Ieff)->_outcome_().set_value((discount_factor_ < 1.0)?(-1.0/(1.0 - discount_factor_)):(-std::numeric_limits<double>::max()));
		}
	}
}
