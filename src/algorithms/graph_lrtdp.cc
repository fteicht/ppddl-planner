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

#include "graph_lrtdp.h"
#include <limits>
#include <cmath>

GraphLRTDP::GraphLRTDP(const Problem& pb, double epsilon, double discount_factor, heuristic_t heuristic_type,
						determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
						unsigned int max_depth)
try : BaseAlgorithm(pb, epsilon, discount_factor),
		GraphHeuristicAlgorithm<GraphStateWT<GraphStateSolveBit> >(pb, epsilon, discount_factor, max_depth, heuristic_type, determinization_type, deterministic_planner_type),
		max_depth_(max_depth)
{
	// Replaces the user-defined maximum depth by the depth at which the cumulated rewards at the initial state won't change
	if (max_depth == 0)
	{
		if (discount_factor_ < 1.0)
			max_depth_ = std::ceil(std::log(epsilon_ * (1.0 - discount_factor_)) / std::log(discount_factor_));
		else
			max_depth_ = 10000;
	}
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphLRTDP::GraphLRTDP");
	throw;
}


void GraphLRTDP::solve_initialize(const PddlState& st)
{
	GraphHeuristicAlgorithm<GraphStateWT<GraphStateSolveBit> >::solve_initialize(st);
	heuristic_->compute(*initial_state_);
	record_value_ = initial_state_->get_value();
}


void GraphLRTDP::solve_progress()
{
	states_stack_t visited_states;
	GraphStateWT<GraphStateSolveBit>* current_state = initial_state_;
	unsigned int cnt = 0;

	while ((cnt < max_depth_) && (!(current_state->solved())))
	{
		cnt++;
		visited_states.push(current_state);

		if (mdp_->goal(*current_state))
			break;

		// Check if the current state is a dead-end (extension of original LRTDP)
		if (((discount_factor_ < 1.0) && (std::fabs((current_state->get_value()) - (-1.0 / (1.0 - discount_factor_))) < epsilon_)) ||
			((discount_factor_ == 1.0) && ((current_state->get_value()) == -std::numeric_limits<double>::max())))
			break;

		// Check if the current state is a dead-end (extension of original LRTDP)
		/*if (current_state->get_transitions().size() == 1)
		{
			if (current_state->get_transitions().begin()->second.size() == 1)
			{
				if (&(current_state->get_transitions().begin()->second.front()->_outcome_()) == current_state)
				{
					current_state->solve();
					current_state->set_value((discount_factor_ < 1.0)?(-1.0 / (1.0 - discount_factor_)):(-std::limits<double>::max()));
					current_state->set_policy(*(current_state->get_transitions().begin()->first));
					break;
				}
			}
		}*/

		if (!(current_state->expanded()))
			initialize(*current_state);

		improve_state_value(*current_state);
		current_state = mdp_->random_transition(*current_state, current_state->get_policy()).first;
	}

	while (!(visited_states.empty()))
	{
		if (!(check_solved(*(visited_states.top()))))
			break;

		visited_states.pop();
	}
}


bool GraphLRTDP::has_converged()
{
	bellman_error_ = std::fabs((initial_state_->get_value()) - record_value_);
	return initial_state_->solved();
}


bool GraphLRTDP::check_solved(GraphStateWT<GraphStateSolveBit>& st)
{
	bool state_solved = true;
	states_stack_t open_stack;
	states_stack_t closed_stack;
	states_set_t open_or_closed_set;

	if (!(st.solved()))
	{
		open_stack.push(&st);
		open_or_closed_set.insert(&st);
	}

	while (!(open_stack.empty()))
	{
		GraphStateWT<GraphStateSolveBit>* current_state = open_stack.top();
		open_stack.pop();
		closed_stack.push(current_state);

		// Not in the original LRTDP version but necessary when constructing the state graph on-the-fly
		if (!(current_state->expanded()))
			initialize(*current_state);

		double value_tmp = current_state->get_value();
		improve_state_value(*current_state);

		if (std::fabs(value_tmp - (current_state->get_value())) > epsilon_)
		{
			state_solved = false;
			continue;
		}

		std::list<GraphTransition<GraphStateWT<GraphStateSolveBit> >*>& effects = current_state->_transitions_().find(&(current_state->get_policy()))->second;

		for (std::list<GraphTransition<GraphStateWT<GraphStateSolveBit> >*>::iterator Ieff = effects.begin() ; Ieff != effects.end() ; ++Ieff)
		{
			GraphStateWT<GraphStateSolveBit>* nst = &((*Ieff)->_outcome_());

			if (!(nst->solved()))
			{
				if (open_or_closed_set.find(nst) == open_or_closed_set.end())
				{
					open_stack.push(nst);
					open_or_closed_set.insert(nst);
				}
			}
		}
	}

	if (state_solved)
	{
		while (!(closed_stack.empty()))
		{
			closed_stack.top()->solve();
			closed_stack.pop();
		}
	}
	else
	{
		while (!(closed_stack.empty()))
		{
			improve_state_value(*(closed_stack.top()));
			closed_stack.pop();
		}
	}

	return state_solved;
}
