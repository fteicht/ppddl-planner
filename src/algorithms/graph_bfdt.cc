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

#include "graph_bfdt.h"

GraphBFDT::GraphBFDT(const Problem& pb, double epsilon, double discount_factor, unsigned int plan_length,
						heuristic_t heuristic_type, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
						unsigned int lookahead)
try : BaseAlgorithm(pb, epsilon, discount_factor),
		GraphHeuristicAlgorithm<GraphStateWT<> >(pb, epsilon, discount_factor, plan_length, heuristic_type, determinization_type, deterministic_planner_type),
		lookahead_(lookahead)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphBFDT::GraphBFDT");
	throw;
}


void GraphBFDT::solve_initialize(const PddlState& st)
{
	reachable_states_.clear();
	reachable_tip_states_.clear();
	optimized_states_.clear();
	GraphHeuristicAlgorithm<GraphStateWT<> >::solve_initialize(st);
	reachable_states_.insert(initial_state_);
	optimized_states_.push_front(initial_state_);
}


void GraphBFDT::solve_progress()
{
	for (states_set_t::iterator Ist = reachable_tip_states_.begin() ; Ist != reachable_tip_states_.end() ; ++Ist)
		initialize(**Ist);

	optimize(optimized_states_);
	compute_reachability();
}


bool GraphBFDT::has_converged()
{
	return (!continue_search_);
}


void GraphBFDT::compute_reachability()
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


void GraphBFDT::compute_lookahead()
{
	if (!continue_search_)
		return;

	states_set_t forward_frontier = reachable_tip_states_;

	for (unsigned int depth = 0 ; depth < lookahead_ ; depth++)
	{
		states_set_t new_forward_frontier;

		for (states_set_t::iterator Ist = forward_frontier.begin() ; Ist != forward_frontier.end() ; ++Ist)
		{
			if (reachable_tip_states_.find(*Ist) != reachable_tip_states_.end())
			{
				initialize(**Ist);
				improve_state_value(**Ist);
				reachable_tip_states_.erase(*Ist);
			}

			std::list<GraphTransition<GraphStateWT<> >*>& effects = (*Ist)->_transitions_().find(&((*Ist)->get_policy()))->second;

			for (std::list<GraphTransition<GraphStateWT<> >*>::iterator Ieff = effects.begin() ; Ieff != effects.end() ; ++Ieff)
			{
				GraphStateWT<>& nst = (*Ieff)->_outcome_();
				std::pair<states_set_t::iterator, bool> res = reachable_states_.insert(&nst);

				if (res.second)
				{
					if (!(nst.expanded()))
						reachable_tip_states_.insert(&nst);

					new_forward_frontier.insert(&nst);
					optimized_states_.push_front(&nst);
				}
			}

			/*if (effects.empty())
				continue;

			double max_probability = effects.front()->get_probability();
			GraphStateWT* nst = &(effects.front()->_outcome_());

			for (std::list<GraphTransition<GraphStateWT>*>::iterator Ieff = ++(effects.begin()) ; Ieff != effects.end() ; ++Ieff)
			{
				if (((*Ieff)->get_probability()) > max_probability)
				{
					max_probability = (*Ieff)->get_probability();
					nst = &((*Ieff)->_outcome_());
				}
			}

			std::pair<states_set_t::iterator, bool> res = reachable_states_.insert(nst);

			if (res.second)
			{
				if (is_tip(*nst))
					reachable_tip_states_.insert(nst);

				new_forward_frontier.insert(nst);
				optimized_states_.push_front(nst);
			}*/
		}

		forward_frontier = new_forward_frontier;
	}
}
