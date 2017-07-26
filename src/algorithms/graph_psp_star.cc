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
#include "graph_psp_star.h"

// CLASS Particle

void Particle::change_state(GraphMDP<GraphStateWT<> >& mdp)
{
	current_state_ = mdp.random_transition(*current_state_, current_state_->get_policy()).first;
}




// CLASS GraphPSPstar

GraphPSPstar::GraphPSPstar(const Problem& pb, double epsilon, double discount_factor, heuristic_t heuristic_type,
							determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
							double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth)
try : BaseAlgorithm(pb, epsilon, discount_factor),
		GraphHeuristicAlgorithm<GraphStateWT<> >(pb, epsilon, discount_factor, trajectories_max_depth, heuristic_type, determinization_type, deterministic_planner_type),
		trajectories_max_depth_(trajectories_max_depth)
{
	set_probabilistic_threshold(probabilistic_threshold);
	particles_.resize(nb_of_particles);
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphPSPstar::GraphPSPstar");
	throw;
}


void GraphPSPstar::set_probabilistic_threshold(double probabilistic_threshold)
{
	if ((probabilistic_threshold < 0.0) || (probabilistic_threshold > 1.0))
		throw BaseException("probabilistic threshold must be in range [0;1]", "GraphPSPstar::set_probabilistic_threshold");

	probabilistic_threshold_ = probabilistic_threshold;
}


void GraphPSPstar::solve_initialize(const PddlState& st)
{
	reachable_states_.clear();
	GraphHeuristicAlgorithm<GraphStateWT<> >::solve_initialize(st);
	reachable_states_.insert(initial_state_);
}


void GraphPSPstar::solve_progress()
{
	optimize(reachable_states_);
	compute_reachability();

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
}


bool GraphPSPstar::has_converged()
{
	return (!continue_search_);
}


//void GraphPSPstar::compute_reachability()
//{
//	//states_set_t new_reachable_states;
//	//new_reachable_states.insert(initial_state_);
//	reachable_states_.clear();
//	reachable_states_.insert(initial_state_);
//	states_set_t reachable_tip_states;
//
//	for (particles_list_t::iterator Ipa = particles_.begin() ; Ipa != particles_.end() ; ++Ipa)
//	{
//		Ipa->set_state(*initial_state_);
//		unsigned int cnt = 0;
//
//		while (cnt < trajectories_max_depth_)
//		{
//			cnt++;
//			Ipa->change_state(*mdp_);
//			//new_reachable_states.insert(Ipa->get_state());
//			reachable_states_.insert(Ipa->get_state());
//
//			if (is_tip(*(Ipa->get_state())))
//			{
//				reachable_tip_states.insert(Ipa->get_state());
//				break;
//			}
//		}
//	}
//
////	states_set_t reachable_tip_states;
////	continue_search_ = false;
////
////	for (states_set_t::const_iterator Ist = new_reachable_states.begin() ; Ist != new_reachable_states.end() ; ++Ist)
////	{
////		if (reachable_states_.find(*Ist) == reachable_states_.end())
////			continue_search_ = true;
////
////		if (tip_states_.find(*Ist) != tip_states_.end())
////			reachable_tip_states.insert(*Ist);
////	}
////
////	reachable_states_ = new_reachable_states;
//
//	for (states_set_t::iterator Ist = reachable_tip_states.begin() ; Ist != reachable_tip_states.end() ; ++Ist)
//		initialize(**Ist);
//}


//void GraphPSPstar::compute_reachability()
//{
//	particles_cnt_t particles_cnt;
//	states_set_t visited_states;
//
//	for (particles_list_t::iterator Ipa = particles_.begin() ; Ipa != particles_.end() ; ++Ipa)
//	{
//		Ipa->set_state(*initial_state_);
//		visited_states.clear();
//		visited_states.insert(initial_state_);
//		unsigned int cnt = 0;
//
//		while ((cnt < trajectories_max_depth_) && (!is_tip(*(Ipa->get_state()))))
//		{
//			cnt++;
//			Ipa->change_state(*mdp_);
//			visited_states.insert(Ipa->get_state());
//		}
//
//		for (states_set_t::const_iterator Ist = visited_states.begin() ; Ist != visited_states.end() ; ++Ist)
//		{
//			std::pair<particles_cnt_t::iterator, bool> res = particles_cnt.insert(std::make_pair(*Ist, 1.0));
//
//			if (!(res.second))
//				res.first->second += 1.0;
//		}
//	}
//
////	states_set_t new_reachable_states;
//	states_set_t reachable_tip_states;
////	continue_search_ = false;
//	reachable_states_.clear();
//	double nb_of_particles = static_cast<double>(particles_.size());
//
//	for (particles_cnt_t::const_iterator Ipc = particles_cnt.begin() ; Ipc != particles_cnt.end() ; ++Ipc)
//	{
//		if (((Ipc->second) / nb_of_particles) > probabilistic_threshold_)
//		{
//			//new_reachable_states.insert(Ipc->first);
//			reachable_states_.insert(Ipc->first);
//
//			//if (reachable_states_.find(Ipc->first) == reachable_states_.end())
//				//continue_search_ = true;
//
//			if (tip_states_.find(Ipc->first) != tip_states_.end())
//				reachable_tip_states.insert(Ipc->first);
//		}
//	}
//
//	//reachable_states_ = new_reachable_states;
//
//	for (states_set_t::iterator Ist = reachable_tip_states.begin() ; Ist != reachable_tip_states.end() ; ++Ist)
//		initialize(**Ist);
//}


void GraphPSPstar::compute_reachability()
{
	states_set_t new_reachable_states;
	new_reachable_states.insert(initial_state_);
	states_set_t forward_frontier;
	forward_frontier.insert(initial_state_);
	states_set_t reachable_tip_states;

	while (!forward_frontier.empty())
	{
		states_set_t new_forward_frontier;

		for (states_set_t::iterator Ist = forward_frontier.begin() ; Ist != forward_frontier.end() ; ++Ist)
		{
			std::list<GraphTransition<GraphStateWT<> >*>& effects = (*Ist)->_transitions_().find(&((*Ist)->get_policy()))->second;

			for (std::list<GraphTransition<GraphStateWT<> >*>::iterator Ieff = effects.begin() ; Ieff != effects.end() ; ++Ieff)
			{
				GraphStateWT<>& nst = (*Ieff)->_outcome_();

				if (new_reachable_states.insert(&nst).second)
				{
					if (nst.expanded())
						new_forward_frontier.insert(&nst);
					else
						reachable_tip_states.insert(&nst);
				}
			}
		}

		forward_frontier = new_forward_frontier;
	}

	if (compute_exit_probability() < probabilistic_threshold_)
		continue_search_ = false;
	else
	{
		continue_search_ = true;
		reachable_states_ = new_reachable_states;

		for (states_set_t::iterator Ist = reachable_tip_states.begin() ; Ist != reachable_tip_states.end() ; ++Ist)
			initialize(**Ist);
	}
}


double GraphPSPstar::compute_exit_probability()
{
	unsigned int nb_exits = 0.0;

	for (particles_list_t::iterator Ipa = particles_.begin() ; Ipa != particles_.end() ; ++Ipa)
	{
		Ipa->set_state(*initial_state_);
		unsigned int cnt = 0;

		while ((cnt < trajectories_max_depth_) && (Ipa->get_state()->expanded()))
		{
			cnt++;
			Ipa->change_state(*mdp_);

			if (reachable_states_.find(Ipa->get_state()) == reachable_states_.end())
			{
				nb_exits++;
				break;
			}
		}
	}

	return (static_cast<double>(nb_exits) / static_cast<double>(particles_.size()));
}
