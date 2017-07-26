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
#include "symbolic_vi.h"

SymbolicVI::SymbolicVI(const Problem& pb, double epsilon, double discount_factor)
try : BaseAlgorithm(pb, epsilon, discount_factor), SymbolicAlgorithm(pb, epsilon, discount_factor)
{
    tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);
}
catch (BaseException& error)
{
	error.push_function_backtrace("SymbolicVI::SymbolicVI");
	throw;
}


SymbolicVI::~SymbolicVI()
{
	record_values_.clear();
}


bool SymbolicVI::policy_defined(const PddlState& st)
{
	return ((mdp_->explored(st))
			&& ((mdp_->policy_defined(mdp_->get_state(st)).get()) != Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager())));
}


void SymbolicVI::solve_initialize(const PddlState& st)
{
	dd_node_ptr new_states = mdp_->add_state(st);
	
	while (new_states != Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()))
		new_states = mdp_->expand(new_states);
	
	if (!(mdp_->get_value_function()))
		mdp_->_value_function_() = dd_node_ptr(Cudd_ReadZero(dd_node_ptr::get_cudd_manager()));
	
	if (!discount_factor_dd_)
		discount_factor_dd_ = dd_node_ptr(Cudd_addConst(dd_node_ptr::get_cudd_manager(), discount_factor_));
}
	

void SymbolicVI::solve_progress()
{
	record_values_.copy(mdp_->_value_function_());
	
	if (mdp_->get_transitions().empty())
		return;
	
	dd_node_ptr primed_value_function(Cudd_addPermute(dd_node_ptr::get_cudd_manager(), mdp_->get_value_function().get(), mdp_->get_unprimed_to_primed_permutation()));
	primed_value_function = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, discount_factor_dd_.get(), primed_value_function.get()));
	mdp_->_value_function_() = compute_action_value(primed_value_function, mdp_->get_transitions().begin()->second);
	
	for (SymbolicMDP::transitions_map_t::const_iterator Itr = ++(mdp_->get_transitions().begin()) ; Itr != mdp_->get_transitions().end() ; ++Itr)
	{
		dd_node_ptr action_value = compute_action_value(primed_value_function, Itr->second);
		mdp_->_value_function_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addMaximum, mdp_->get_value_function().get(), action_value.get()));
	}
}


bool SymbolicVI::has_converged()
{
	compute_bellman_error(record_values_, mdp_->get_value_function());
	
	if (bellman_error_ < epsilon_)
	{
		dd_node_ptr primed_value_function(Cudd_addPermute(dd_node_ptr::get_cudd_manager(), record_values_.get(), mdp_->get_unprimed_to_primed_permutation()));
		compute_policy(mdp_->_policy_(), mdp_->_value_function_(), primed_value_function, mdp_->get_transitions());
		return true;
	}
	else
		return false;
}
