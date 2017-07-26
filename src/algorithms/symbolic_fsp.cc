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

#include "symbolic_fsp.h"

SymbolicFSP::SymbolicFSP(const Problem& pb, double epsilon, double discount_factor, unsigned int plan_length, heuristic_t heuristic_type,
							determinization_t determinization_type, deterministic_planner_t deterministic_planner_type)
try : BaseAlgorithm(pb, epsilon, discount_factor),
		SymbolicHeuristicAlgorithm(pb, epsilon, discount_factor, plan_length, heuristic_type, determinization_type, deterministic_planner_type)
{
	policy_deterministic_transitions_ = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	forward_reachable_states_ = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	backward_reachable_states_ = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	states_frontier_ = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	mdp_->connect_bdd(policy_deterministic_transitions_);
	mdp_->connect_bdd(forward_reachable_states_);
	mdp_->connect_bdd(backward_reachable_states_);
	mdp_->connect_bdd(states_frontier_);
}
catch (BaseException& error)
{
	error.push_function_backtrace("SymbolicFSP::SymbolicFSP");
	throw;
}


SymbolicFSP::~SymbolicFSP()
{
	mdp_->disconnect_bdd(policy_deterministic_transitions_);
	mdp_->disconnect_bdd(forward_reachable_states_);
	mdp_->disconnect_bdd(backward_reachable_states_);
	mdp_->disconnect_bdd(states_frontier_);
}


void SymbolicFSP::solve_initialize(const PddlState& st)
{
	SymbolicHeuristicAlgorithm::solve_initialize(st);
	forward_reachable_states_.copy(initial_state_);
	states_frontier_.copy(initial_state_);
	policy_deterministic_transitions_ = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
}


void SymbolicFSP::solve_progress()
{
	compute_backward_reachability();
	optimize(backward_reachable_states_);
	compute_forward_reachability();
}


bool SymbolicFSP::has_converged()
{
	return (states_frontier_.get() == Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
}


void SymbolicFSP::compute_backward_reachability()
{
	dd_node_ptr forward_reachable_states_minus_frontier(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), forward_reachable_states_.get(), states_frontier_.get()));
	policy_deterministic_transitions_ = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), policy_deterministic_transitions_.get(), forward_reachable_states_minus_frontier.get()));
	backward_reachable_states_.copy(states_frontier_);
	dd_node_ptr backward_frontier;
	backward_frontier.copy(states_frontier_);

	while (backward_frontier.get() != Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()))
	{
		dd_node_ptr previous_states(Cudd_bddPermute(dd_node_ptr::get_cudd_manager(), backward_frontier.get(), mdp_->get_unprimed_to_primed_permutation()));
		previous_states = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), policy_deterministic_transitions_.get(), previous_states.get()));
		previous_states = dd_node_ptr(Cudd_bddExistAbstract(dd_node_ptr::get_cudd_manager(), previous_states.get(), mdp_->get_primed_variables_bdd_cube().get()));
		backward_frontier = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), backward_reachable_states_.get(), previous_states.get()));
		backward_frontier = dd_node_ptr(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), backward_frontier.get(), previous_states.get()));
		backward_reachable_states_ = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), backward_reachable_states_.get(), backward_frontier.get()));
	}
}


void SymbolicFSP::compute_forward_reachability()
{
	dd_node_ptr new_reachable_states;
	new_reachable_states.copy(initial_state_);
	dd_node_ptr forward_frontier;
	forward_frontier.copy(initial_state_);
	states_frontier_ = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	policy_deterministic_transitions_ = compute_policy_deterministic_transitions(forward_reachable_states_);

	while (forward_frontier.get() != Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()))
	{
		dd_node_ptr next_states(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), forward_frontier.get(), policy_deterministic_transitions_.get()));
		next_states = dd_node_ptr(Cudd_bddExistAbstract(dd_node_ptr::get_cudd_manager(), next_states.get(), mdp_->get_unprimed_variables_bdd_cube().get()));
		next_states = dd_node_ptr(Cudd_bddPermute(dd_node_ptr::get_cudd_manager(), next_states.get(), mdp_->get_primed_to_unprimed_permutation()));
		forward_frontier = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), next_states.get(), new_reachable_states.get()));
		forward_frontier = dd_node_ptr(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), next_states.get(), forward_frontier.get()));
		dd_node_ptr new_states_frontier = forward_frontier;
		new_reachable_states = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), new_reachable_states.get(), new_states_frontier.get()));
		forward_frontier = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), new_states_frontier.get(), forward_reachable_states_.get()));
		new_states_frontier = dd_node_ptr(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), new_states_frontier.get(), forward_frontier.get()));
		states_frontier_ = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), states_frontier_.get(), new_states_frontier.get()));
	}

	forward_reachable_states_ = new_reachable_states;
	dd_node_ptr reachable_tip_states = tips(states_frontier_);
	initialize(reachable_tip_states);
}
