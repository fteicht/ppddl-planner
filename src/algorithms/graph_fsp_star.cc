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

#include "graph_fsp_star.h"

GraphFSPstar::GraphFSPstar(const Problem& pb, double epsilon, double discount_factor, unsigned int plan_length, heuristic_t heuristic_type,
							determinization_t determinization_type, deterministic_planner_t deterministic_planner_type)
try : BaseAlgorithm(pb, epsilon, discount_factor),
		GraphHeuristicAlgorithm<GraphStateWT<> >(pb, epsilon, discount_factor, plan_length, heuristic_type, determinization_type, deterministic_planner_type)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphFSPstar::GraphFSPstar");
	throw;
}


void GraphFSPstar::solve_initialize(const PddlState& st)
{
	reachable_states_.clear();
	reachable_tip_states_.clear();
	optimized_states_.clear();
	GraphHeuristicAlgorithm<GraphStateWT<> >::solve_initialize(st);
	reachable_states_.insert(initial_state_);
	optimized_states_.push_front(initial_state_);
}


void GraphFSPstar::solve_progress()
{
	for (states_set_t::iterator Ist = reachable_tip_states_.begin() ; Ist != reachable_tip_states_.end() ; ++Ist)
		initialize(**Ist);

	optimize(optimized_states_);
	compute_reachability();
}


bool GraphFSPstar::has_converged()
{
	return (!continue_search_);
}


void GraphFSPstar::compute_reachability()
{
	states_set_t new_reachable_states;
	new_reachable_states.insert(initial_state_);
	states_set_t forward_frontier;
	forward_frontier.insert(initial_state_);
	reachable_tip_states_.clear();
	optimized_states_.clear();
	optimized_states_.push_front(initial_state_);
	continue_search_ = false;

	while (!forward_frontier.empty())
	{
		states_set_t new_forward_frontier;

		for (states_set_t::iterator Ist = forward_frontier.begin() ; Ist != forward_frontier.end() ; ++Ist)
		{
			std::list<GraphTransition<GraphStateWT<> >*>& effects = (*Ist)->_transitions_().find(&((*Ist)->get_policy()))->second;

			for (std::list<GraphTransition<GraphStateWT<> >*>::iterator Ieff = effects.begin() ; Ieff != effects.end() ; ++Ieff)
			{
				GraphStateWT<>& nst = (*Ieff)->_outcome_();
				std::pair<states_set_t::iterator, bool> res = new_reachable_states.insert(&nst);

				if (res.second)
				{
					optimized_states_.push_front(&nst);

					if (reachable_states_.find(&nst) != reachable_states_.end())
						new_forward_frontier.insert(&nst);
					else
					{
						continue_search_ = true;

						if (nst.expanded())
							new_forward_frontier.insert(&nst);
						else
							reachable_tip_states_.insert(&nst);
					}
				}
			}
		}

		forward_frontier = new_forward_frontier;
	}

	reachable_states_ = new_reachable_states;
}
