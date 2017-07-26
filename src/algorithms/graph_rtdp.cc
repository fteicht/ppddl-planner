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
#include "graph_rtdp.h"

GraphRTDP::GraphRTDP(const Problem& pb, double epsilon, double discount_factor, heuristic_t heuristic_type,
						determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
						unsigned int max_depth)
try : BaseAlgorithm(pb, epsilon, discount_factor),
		GraphHeuristicAlgorithm<GraphStateWT<> >(pb, epsilon, discount_factor, max_depth, heuristic_type, determinization_type, deterministic_planner_type),
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
	error.push_function_backtrace("GraphRTDP::GraphRTDP");
	throw;
}


void GraphRTDP::solve_initialize(const PddlState& st)
{
	record_values_.clear();
	GraphHeuristicAlgorithm<GraphStateWT<> >::solve_initialize(st);
	record_values_.insert(std::make_pair(initial_state_, initial_state_->get_value()));
}


void GraphRTDP::solve_progress()
{
	GraphStateWT<>* current_state = initial_state_;
	unsigned int cnt = 0;

	for (states_values_t::iterator Ist = record_values_.begin() ; Ist != record_values_.end() ; ++Ist)
		Ist->second = Ist->first->get_value();

	while ((cnt < max_depth_) && (!(mdp_->goal(*current_state))))
	{
		cnt++;

		if (!(current_state->expanded()))
		{
			initialize(*current_state);
			record_values_.insert(std::make_pair(current_state, current_state->get_value()));
		}

		improve_state_value(*current_state);
		current_state = mdp_->random_transition(*current_state, current_state->get_policy()).first;
	}
}


bool GraphRTDP::has_converged()
{
	bellman_error_ = 0.0;

	for (states_values_t::const_iterator Ist = record_values_.begin() ; Ist != record_values_.end() ; ++Ist)
		bellman_error_ = std::max(bellman_error_, std::fabs((Ist->first->get_value()) - (Ist->second)));

	return (bellman_error_ < epsilon_);
}
