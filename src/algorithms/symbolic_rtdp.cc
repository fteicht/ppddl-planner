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
#include "symbolic_rtdp.h"

SymbolicRTDP::SymbolicRTDP(const Problem& pb, double epsilon, double discount_factor, heuristic_t heuristic_type,
							determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
							unsigned int max_depth)
try : BaseAlgorithm(pb, epsilon, discount_factor),
		SymbolicHeuristicAlgorithm(pb, epsilon, discount_factor, max_depth, heuristic_type, determinization_type, deterministic_planner_type),
		max_depth_(max_depth)
{
	generalized_states_ = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));;
	record_values_ = dd_node_ptr(Cudd_ReadZero(dd_node_ptr::get_cudd_manager()));
	mdp_->connect_bdd(generalized_states_);
	mdp_->connect_add(record_values_);

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
	error.push_function_backtrace("SymbolicRTDP::SymbolicRTDP");
	throw;
}


SymbolicRTDP::~SymbolicRTDP()
{
	mdp_->disconnect_bdd(generalized_states_);
	mdp_->disconnect_add(record_values_);
}


void SymbolicRTDP::solve_initialize(const PddlState& st)
{
	SymbolicHeuristicAlgorithm::solve_initialize(st);
	record_values_.copy(mdp_->get_value_function());
}


void SymbolicRTDP::solve_progress()
{
	dd_node_ptr current_state;
	current_state.copy(initial_state_);
	unsigned int cnt = 0;
	record_values_.copy(mdp_->get_value_function());

	while ((cnt < max_depth_) &&
			(mdp_->goals(current_state) == Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager())))
	{
		cnt++;

		if (tips(current_state) != Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()))
			initialize(current_state);

		generalize(current_state);
		improve_generalized_states();
		current_state = mdp_->random_transition(current_state, mdp_->get_policy(current_state)).first;
	}
}


bool SymbolicRTDP::has_converged()
{
	compute_bellman_error(record_values_, mdp_->get_value_function());
	return (bellman_error_ < epsilon_);
}


void SymbolicRTDP::generalize(const dd_node_ptr& st)
{
	double st_val = mdp_->get_value(st);
	double delta = std::exp(std::log(epsilon_) / 2.0);
	generalized_states_ = dd_node_ptr(Cudd_addBddInterval(dd_node_ptr::get_cudd_manager(), mdp_->get_value_function().get(), st_val - delta, st_val + delta));
	generalized_states_ = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), generalized_states_.get(), explored_states_.get()));
	dd_node_ptr and_dd(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), generalized_states_.get(), tip_states_.get()));
	generalized_states_ = dd_node_ptr(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), generalized_states_.get(), and_dd.get()));
}


void SymbolicRTDP::improve_generalized_states()
{
	dd_node_ptr states_fringe;
	compute_fringe(generalized_states_, states_fringe);
	dd_node_ptr generalized_states_or_fringe(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), generalized_states_.get(), states_fringe.get()));
	generalized_states_or_fringe = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), generalized_states_or_fringe.get()));
	dd_node_ptr primed_local_value_function(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, mdp_->get_value_function().get(), generalized_states_or_fringe.get()));
	primed_local_value_function = dd_node_ptr(Cudd_addPermute(dd_node_ptr::get_cudd_manager(), primed_local_value_function.get(), mdp_->get_unprimed_to_primed_permutation()));

	SymbolicMDP::transitions_map_t local_transitions;
	compute_local_transitions(generalized_states_, local_transitions);

	dd_node_ptr local_policy;
	dd_node_ptr local_value_function;
	compute_policy(local_policy, local_value_function, primed_local_value_function, local_transitions);

	// Updates the global value function and the global policy
	dd_node_ptr generalized_states_cmpl(Cudd_Not(generalized_states_.get()));
	generalized_states_cmpl = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), generalized_states_cmpl.get()));
	mdp_->_value_function_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, mdp_->get_value_function().get(), generalized_states_cmpl.get()));
	mdp_->_value_function_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, mdp_->get_value_function().get(), local_value_function.get()));
	mdp_->_policy_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, mdp_->get_policy().get(), generalized_states_cmpl.get()));
	mdp_->_policy_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, mdp_->get_policy().get(), local_policy.get()));
}
