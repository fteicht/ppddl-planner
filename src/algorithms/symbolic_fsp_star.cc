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

#include "symbolic_fsp_star.h"

SymbolicFSPstar::SymbolicFSPstar(const Problem& pb, double epsilon, double discount_factor, unsigned int plan_length, heuristic_t heuristic_type,
									determinization_t determinization_type, deterministic_planner_t deterministic_planner_type)
try : BaseAlgorithm(pb, epsilon, discount_factor),
		SymbolicHeuristicAlgorithm(pb, epsilon, discount_factor, plan_length, heuristic_type, determinization_type, deterministic_planner_type)
{
	reachable_states_ = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	mdp_->connect_bdd(reachable_states_);
}
catch (BaseException& error)
{
	error.push_function_backtrace("SymbolicFSPstar::SymbolicFSPstar");
	throw;
}


SymbolicFSPstar::~SymbolicFSPstar()
{
	mdp_->disconnect_bdd(reachable_states_);
}


void SymbolicFSPstar::solve_initialize(const PddlState& st)
{
	SymbolicHeuristicAlgorithm::solve_initialize(st);
	reachable_states_.copy(initial_state_);
}


void SymbolicFSPstar::solve_progress()
{
	optimize(reachable_states_);
	compute_reachability();
}


bool SymbolicFSPstar::has_converged()
{
	return (!continue_search_);
}


void SymbolicFSPstar::compute_reachability()
{
	dd_node_ptr new_reachable_states;
	new_reachable_states.copy(initial_state_);
	dd_node_ptr forward_frontier;
	forward_frontier.copy(initial_state_);
	dd_node_ptr reachable_tip_states(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	dd_node_ptr policy_deterministic_transitions = compute_policy_deterministic_transitions(explored_states_);

	while (forward_frontier.get() != Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()))
	{
		dd_node_ptr next_states(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), forward_frontier.get(), policy_deterministic_transitions.get()));
		next_states = dd_node_ptr(Cudd_bddExistAbstract(dd_node_ptr::get_cudd_manager(), next_states.get(), mdp_->get_unprimed_variables_bdd_cube().get()));
		next_states = dd_node_ptr(Cudd_bddPermute(dd_node_ptr::get_cudd_manager(), next_states.get(), mdp_->get_primed_to_unprimed_permutation()));
		forward_frontier = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), next_states.get(), new_reachable_states.get()));
		forward_frontier = dd_node_ptr(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), next_states.get(), forward_frontier.get()));
		dd_node_ptr new_states_frontier = forward_frontier;
		new_reachable_states = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), new_reachable_states.get(), new_states_frontier.get()));

		forward_frontier = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), new_states_frontier.get(), tip_states_.get()));
		reachable_tip_states = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), reachable_tip_states.get(), forward_frontier.get()));
		forward_frontier = dd_node_ptr(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), new_states_frontier.get(), forward_frontier.get()));
	}

	dd_node_ptr and_dd(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), new_reachable_states.get(), reachable_states_.get()));
	continue_search_ = (and_dd.get() != new_reachable_states.get());
	reachable_states_ = new_reachable_states;
	initialize(reachable_tip_states);
}
