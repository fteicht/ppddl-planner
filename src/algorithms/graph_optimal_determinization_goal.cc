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

#include <cmath>

#include "graph_optimal_determinization_goal.h"
#include "models/base_determinization.h"
#include "planners/deterministic_planner.h"


GraphOptimalDeterminizationGoal::GraphOptimalDeterminizationGoal(const Problem& pb, double epsilon, double discount_factor,
		heuristic_t heuristic_type, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth)
try : BaseAlgorithm(pb, epsilon, discount_factor),
		GraphHeuristicAlgorithm<GraphStateWT<> >(pb, epsilon, discount_factor, trajectories_max_depth, heuristic_type, determinization_type, deterministic_planner_type),
		DeterminizationAlgorithm(pb, epsilon, discount_factor, trajectories_max_depth, determinization_type, deterministic_planner_type),
		ParticleAlgorithm<GraphStateWT<>*>(pb, epsilon, discount_factor, probabilistic_threshold, nb_of_particles, trajectories_max_depth)//, continue_search_(true)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphOptimalDeterminizationGoal::GraphOptimalDeterminizationGoal");
	throw;
}


void GraphOptimalDeterminizationGoal::solve_initialize(const PddlState& st)
{
	reachable_states_.clear();
	GraphHeuristicAlgorithm<GraphStateWT<> >::solve_initialize(st);
	reachable_states_.insert(initial_state_);
}


void GraphOptimalDeterminizationGoal::solve_progress()
{
	optimize(reachable_states_);
	compute_reachability();
}


void GraphOptimalDeterminizationGoal::compute_reachability()
{
	exit_probability_ = 0.0;
	double nb_particles_inv = 1.0 / static_cast<double>(particles_.size());
	states_set_t reachable_tip_states;
	states_set_t new_reachable_states;
	new_reachable_states.insert(initial_state_);

	for (particles_list_t::iterator Ip = particles_.begin() ; Ip != particles_.end() ; ++Ip)
	{
		Ip->set_state(initial_state_);
		unsigned int cnt = 0;
		bool inside = true;

		while ((cnt < trajectories_max_depth_) && (!(mdp_->goal(*(Ip->get_state())))))
		{
			if (inside && (reachable_states_.find(Ip->get_state()) == reachable_states_.end()))
			{
				exit_probability_ += nb_particles_inv;
				inside = false;
			}

			if (!(Ip->get_state()->expanded()))
			{
				reachable_tip_states.insert(Ip->get_state());
				break;
			}

			Ip->change_state(*mdp_);
			new_reachable_states.insert(Ip->get_state());
			cnt++;
		}
	}

	reachable_states_ = new_reachable_states;

	for (states_set_t::iterator Ist = reachable_tip_states.begin() ; Ist != reachable_tip_states.end() ; ++Ist)
		generate_trajectory(**Ist);
}


//void GraphOptimalDeterminizationGoal::solve_progress()
//{
//	std::list<double> record_values(reachable_states_.size());
//	std::list<double>::iterator Ivl = record_values.begin();
//
//	for (states_set_t::const_iterator Ist = reachable_states_.begin() ; Ist != reachable_states_.end() ; ++Ist)
//	{
//		*Ivl = (*Ist)->get_value();
//		++Ivl;
//	}
//
//	optimize(reachable_states_);
//
//	bellman_error_ = 0.0;
//	Ivl = record_values.begin();
//
//	for (states_set_t::const_iterator Ist = reachable_states_.begin() ; Ist != reachable_states_.end() ; ++Ist)
//	{
//		bellman_error_ = std::max(bellman_error_, std::fabs((*Ivl) - ((*Ist)->get_value())));
//		++Ivl;
//	}
//
//	if (bellman_error_ < epsilon_)
//		continue_search_ = false;
//	else
//	{
//		continue_search_ = true;
//		compute_reachability();
//	}
//}


bool GraphOptimalDeterminizationGoal::has_converged()
{
	extern int verbosity; // defined in main.cc

	if (verbosity >= 1)
	{
		//std::cerr << "Bellman error: " << bellman_error_ << std::endl;
		std::cerr << "Exit probability: " << exit_probability_ << std::endl;
		std::cerr << dynamic_cast<const GraphMDP<GraphStateWT<> >&>(get_mdp()).get_states().size() << " explored states" << std::endl;
		std::cerr << reachable_states_.size() << " reachable states" << std::endl;
	}

	//return (!continue_search_);
	return (exit_probability_ < probabilistic_threshold_);
}


//void GraphOptimalDeterminizationGoal::compute_reachability()
//{
//	reachable_states_.clear();
//	reachable_states_.insert(initial_state_);
//	states_set_t forward_frontier;
//	forward_frontier.insert(initial_state_);
//	states_set_t reachable_tip_states;
//
//	while (!forward_frontier.empty())
//	{
//		states_set_t new_forward_frontier;
//
//		for (states_set_t::iterator Ist = forward_frontier.begin() ; Ist != forward_frontier.end() ; ++Ist)
//		{
//			std::list<GraphTransition<GraphStateWT>*>& effects = (*Ist)->_transitions_().find(&((*Ist)->get_policy()))->second;
//
//			for (std::list<GraphTransition<GraphStateWT>*>::iterator Ieff = effects.begin() ; Ieff != effects.end() ; ++Ieff)
//			{
//				GraphStateWT& nst = (*Ieff)->_outcome_();
//				std::pair<states_set_t::iterator, bool> res = reachable_states_.insert(&nst);
//
//				if (res.second)
//				{
//					if (!is_tip(nst))
//						new_forward_frontier.insert(&nst);
//					else
//						reachable_tip_states.insert(&nst);
//				}
//			}
//		}
//
//		forward_frontier = new_forward_frontier;
//	}
//
//	for (states_set_t::iterator Ist = reachable_tip_states.begin() ; Ist != reachable_tip_states.end() ; ++Ist)
//		generate_trajectory(**Ist);
//}


void GraphOptimalDeterminizationGoal::generate_trajectory(GraphStateWT<>& st)
{
	DeterministicPlanner::plan_t plan;
	determinization_->make_problem(st.get_pddl_state().atoms(), st.get_pddl_state().values(), tb_->get_problem().goal());
	deterministic_planner_->solve(plan);
	GraphStateWT<>* current_st = &st;
	initialize(*current_st);

	for (DeterministicPlanner::plan_t::const_iterator Ip = plan.begin() ; Ip != plan.end() ; ++Ip)
	{
		PddlState outcome;
		const Action* action;
		determinization_->effect(*tb_, current_st->get_pddl_state(), **Ip, outcome, action);
		current_st->set_policy(*action);
		current_st = &(mdp_->_state_(outcome));
		reachable_states_.insert(current_st);

		if (!(current_st->expanded()))
			initialize(*current_st);
		//else
			//break;
	}
}
