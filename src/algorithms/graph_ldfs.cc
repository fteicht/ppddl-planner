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

#include "graph_ldfs.h"
#include <algorithm>
#include <limits>
#include <cmath>

GraphLDFS::GraphLDFS(const Problem& pb, double epsilon, double discount_factor, unsigned int plan_length, heuristic_t heuristic_type,
						determinization_t determinization_type, deterministic_planner_t deterministic_planner_type)
try : BaseAlgorithm(pb, epsilon, discount_factor),
		GraphHeuristicAlgorithm<GraphStateWT<GraphStateSolveBitConnection> >(pb, epsilon, discount_factor, plan_length, heuristic_type, determinization_type, deterministic_planner_type)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphLDFS::GraphLDFS");
	throw;
}


void GraphLDFS::solve_initialize(const PddlState& st)
{
	GraphHeuristicAlgorithm<GraphStateWT<GraphStateSolveBitConnection> >::solve_initialize(st);
	heuristic_->compute(*initial_state_);
	record_value_ = initial_state_->get_value();
	connected_states_ = states_stack_t();
	active_states_.clear();
}


void GraphLDFS::solve_progress()
{
	ldfs(*initial_state_, 0);
	active_states_.clear();
}


bool GraphLDFS::has_converged()
{
	bellman_error_ = std::fabs((initial_state_->get_value()) - record_value_);
	return (initial_state_->solved());
}

bool GraphLDFS::ldfs(GraphStateWT<GraphStateSolveBitConnection>& st, unsigned long int idx)
{
	// Check if the current state is solved
	if (st.solved())
		return true;

	// Check if the current state is a goal
	if (mdp_->goal(st))
	{
		st.solve();
		return true;
	}

	// Expand the state if it is not yet expanded (tip node)
	if (!(st.expanded())) {
		initialize(st);
//		improve_state_value(st); // not in original LDFS
//		return false; // not in original LDFS
	}

	// Check if the current state is a dead-end (extension of original LDFS)
	/*if (((discount_factor_ == 1.0) && (st.get_value() == -std::numeric_limits<double>::max())) ||
			((discount_factor_ < 1.0) && (std::fabs((st.get_value()) - (-1.0 / (1.0 - discount_factor_))) < epsilon_)))
	{
		st.solve();
		//improve_state_value(st);
		st.set_value((discount_factor_ < 1.0)?(-1.0 / (1.0 - discount_factor_)):(-std::numeric_limits<double>::max()));//
		st.set_policy(*(st.get_transitions().begin()->first));//
		st.set_idx(GraphStateConnection::INFINITE_INDEX);
		st.set_low(GraphStateConnection::INFINITE_INDEX);
		return true;
	}*/

	// Check if the current state is active
	if (active_states_.find(&st) != active_states_.end())
		return false;

	connected_states_.push(&st);
	st.set_idx(idx);
	st.set_low(idx);
	idx++;

	improve_state_value(st);
	bool flag = false;

	for (std::map<const Action*, std::list<GraphTransition<GraphStateWT<GraphStateSolveBitConnection> >*> >::iterator Iac = st._transitions_().begin() ; Iac != st._transitions_().end() ; ++Iac)
	{
		if (std::fabs(compute_q_value(Iac->second) - (st.get_value())) > epsilon_)
			continue;

		active_states_.insert(&st);
		flag = true;

		for (std::list<GraphTransition<GraphStateWT<GraphStateSolveBitConnection> >*>::iterator Ieff = Iac->second.begin() ; Ieff != Iac->second.end() ; ++Ieff)
		{
			GraphStateWT<GraphStateSolveBitConnection>& nst = (*Ieff)->_outcome_();

			if (nst.get_idx() == GraphStateConnection::INFINITE_INDEX)
			{
				flag = ldfs(nst, idx) && flag;
				st.set_low(std::min(st.get_low(), nst.get_low()));
			}
			else if (active_states_.find(&nst) != active_states_.end())
				st.set_low(std::min(st.get_low(), nst.get_idx()));
		}

		flag = flag && ((std::fabs(compute_q_value(Iac->second) - (st.get_value())) <= epsilon_)?true:false);

		if (flag)
			break;

		while ((!(connected_states_.empty())) && ((connected_states_.top()->get_idx()) > st.get_idx()))
		{
			connected_states_.top()->set_idx(GraphStateConnection::INFINITE_INDEX);
			connected_states_.top()->set_low(GraphStateConnection::INFINITE_INDEX);
			connected_states_.pop();
		}
	}

	if (!flag)
	{
		improve_state_value(st);
		st.set_idx(GraphStateConnection::INFINITE_INDEX);
		st.set_low(GraphStateConnection::INFINITE_INDEX);
		connected_states_.pop();
	}
	else if (st.get_low() == st.get_idx())
	{
		while ((!(connected_states_.empty())) && ((connected_states_.top()->get_idx()) >= st.get_idx()))
		{
			st.solve();
			connected_states_.top()->set_idx(GraphStateConnection::INFINITE_INDEX);
			connected_states_.top()->set_low(GraphStateConnection::INFINITE_INDEX);
			connected_states_.pop();
		}
	}

	return flag;
}
