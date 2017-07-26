/* 
   ppddl-planner - client for IPPC'08

   Copyright (C) 2012 Florent Teichteil-Koenigsbuch and Guillaume Infantes and Ugur Kuter

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
#include <limits>
#include <sstream>
#include <cuddInt.h>
#include "symbolic_gco.h"


SymbolicGCO::SymbolicGCO(const Problem& pb, double epsilon)
try : BaseAlgorithm(pb, epsilon, 1.0), GoalStatisticsAlgorithm(pb, epsilon),
      SymbolicAlgorithm(pb, epsilon, 1.0)
{
    tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);
}
catch (BaseException& error)
{
	error.push_function_backtrace("SymbolicGCO::SymbolicGCO");
	throw;
}


double SymbolicGCO::goal_probability(const PddlState& st) const
{
	dd_node_ptr dds;
	
	try
	{
	    dds = mdp_->get_state(st);
	}
	catch (const BaseException& e)
	{
		return 0.0;
	}
	
	dd_node_ptr cudd_state(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), dds.get()));
	DdNode* probability_dd_node = Cudd_addEvalConst(dd_node_ptr::get_cudd_manager(), cudd_state.get(), mdp_->get_probability_function().get());

	if (probability_dd_node == DD_NON_CONSTANT)
	{
		std::ostringstream state_str;
		state_str << "state " << st << " is incomplete (it is a set of states)";
		throw BaseException(state_str.str(), "SymbolicGCO::goal_probability");
	}

	dd_node_ptr probability_dd(probability_dd_node);
	return Cudd_V(probability_dd.get());
}


double SymbolicGCO::goal_cost(const PddlState& st) const
{
	dd_node_ptr dds;
	
    try
	{
	    dds = mdp_->get_state(st);
	}
	catch (const BaseException& e)
	{
		return 0.0;
	}
	
	dd_node_ptr cudd_state(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), dds.get()));
	DdNode* cost_dd_node = Cudd_addEvalConst(dd_node_ptr::get_cudd_manager(), cudd_state.get(), mdp_->get_value_function().get());

	if (cost_dd_node == DD_NON_CONSTANT)
	{
		std::ostringstream state_str;
		state_str << "state " << st << " is incomplete (it is a set of states)";
		throw BaseException(state_str.str(), "SymbolicGCO::goal_cost");
	}

	dd_node_ptr cost_dd(cost_dd_node);
	return Cudd_V(cost_dd.get());
}


bool SymbolicGCO::policy_defined(const PddlState& st)
{
	return ((mdp_->explored(st))
			&& ((mdp_->policy_defined(mdp_->get_state(st)).get()) != Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager())));
}


void SymbolicGCO::solve_initialize(const PddlState& st)
{
	// generate all reachable states
	dd_node_ptr new_states = mdp_->add_state(st);
	
	while (new_states != Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()))
		new_states = mdp_->expand(new_states);
	
	if (!(mdp_->get_value_function()))
		mdp_->_value_function_() = dd_node_ptr(Cudd_ReadZero(dd_node_ptr::get_cudd_manager()));

	if (!(mdp_->get_probability_function()))
		mdp_->_probability_function_() = dd_node_ptr(Cudd_ReadZero(dd_node_ptr::get_cudd_manager()));
	
	if (!discount_factor_dd_)
		discount_factor_dd_ = dd_node_ptr(Cudd_addConst(dd_node_ptr::get_cudd_manager(), discount_factor_));
	
	// Compute deterministic transition graph
	dd_node_ptr deterministic_transitions(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	
	for (SymbolicMDP::transitions_map_t::const_iterator Itr = mdp_->get_transitions().begin() ; Itr != mdp_->get_transitions().end() ; ++Itr)
	{
		dd_node_ptr deterministic_action_transitions(Cudd_addBddPattern(dd_node_ptr::get_cudd_manager(), Itr->second.get_probabilities().get()));
		deterministic_transitions = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), deterministic_transitions.get(), deterministic_action_transitions.get()));
	}
	
	// backward restrict states to the one reaching the goal with a positive probability
	dd_node_ptr states_frontier(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), mdp_->get_goal_states().get(), mdp_->get_states().get()));
	mdp_->_probability_function_() = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), states_frontier.get()));
	restricted_states_.copy(states_frontier);
	
	while (states_frontier.get() != Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()))
	{
		dd_node_ptr previous_states(Cudd_bddPermute(dd_node_ptr::get_cudd_manager(), states_frontier.get(), mdp_->get_unprimed_to_primed_permutation()));
		previous_states = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), deterministic_transitions.get(), previous_states.get()));
		previous_states = dd_node_ptr(Cudd_bddExistAbstract(dd_node_ptr::get_cudd_manager(), previous_states.get(), mdp_->get_primed_variables_bdd_cube().get()));
		states_frontier = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), restricted_states_.get(), previous_states.get()));
		states_frontier = dd_node_ptr(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), states_frontier.get(), previous_states.get()));
		restricted_states_ = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), restricted_states_.get(), states_frontier.get()));
	}
	
	// Restrict all transitions to reachable states
	dd_node_ptr states_add(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), restricted_states_.get()));
	dd_node_ptr primed_states_add(Cudd_addPermute(dd_node_ptr::get_cudd_manager(), states_add.get(), mdp_->get_unprimed_to_primed_permutation()));
	
	for (SymbolicMDP::transitions_map_t::iterator Itr = mdp_->_transitions_().begin() ; Itr != mdp_->_transitions_().end() ; ++Itr)
	{
		//Itr->second._policy_mask_() = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), Itr->second.get_policy_mask().get(), restricted_states_.get()));
		Itr->second._value_mask_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, Itr->second.get_value_mask().get(), states_add.get()));
		Itr->second._probabilities_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, Itr->second.get_probabilities().get(), states_add.get()));
		Itr->second._probabilities_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, Itr->second.get_probabilities().get(), primed_states_add.get()));
		Itr->second._rewards_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, Itr->second.get_rewards().get(), states_add.get()));
		Itr->second._rewards_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, Itr->second.get_rewards().get(), primed_states_add.get()));
	}
	
	record_values_ = dd_node_ptr(Cudd_ReadZero(dd_node_ptr::get_cudd_manager()));
	record_probabilities_ = dd_node_ptr(Cudd_ReadZero(dd_node_ptr::get_cudd_manager()));
	probability_converged_ = false;
}


void SymbolicGCO::solve_progress()
{
	bellman_error_ = 0.0;
    
    if (!probability_converged_)
    {
        dd_node_ptr primed_probability_function(Cudd_addPermute(dd_node_ptr::get_cudd_manager(), mdp_->get_probability_function().get(), mdp_->get_unprimed_to_primed_permutation()));
		record_probabilities_ = compute_action_probability(primed_probability_function, mdp_->get_transitions().begin()->second);

		for (SymbolicMDP::transitions_map_t::const_iterator Itr = ++(mdp_->get_transitions().begin()) ; Itr != mdp_->get_transitions().end() ; ++Itr)
		{
			dd_node_ptr action_probability = compute_action_probability(primed_probability_function, Itr->second);
			record_probabilities_ = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addMaximum, record_probabilities_.get(), action_probability.get()));
		}
		
		compute_bellman_error(record_probabilities_, mdp_->get_probability_function());
		mdp_->_probability_function_().copy(record_probabilities_);
		        
        if (bellman_error_ < epsilon_)
        {
            probability_converged_ = true;
            bellman_error_ = std::numeric_limits<double>::max();
			
			// Transitions' transformation
			post_probability_function_ = dd_node_ptr(Cudd_addPermute(dd_node_ptr::get_cudd_manager(), mdp_->get_probability_function().get(), mdp_->get_unprimed_to_primed_permutation()));
			pre_probability_function_ = dd_node_ptr(Cudd_Not(restricted_states_.get()));
			pre_probability_function_ = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), pre_probability_function_.get()));
			pre_probability_function_ = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, pre_probability_function_.get(), mdp_->get_probability_function().get()));
		}
    }
    else
    {
		dd_node_ptr primed_value_function(Cudd_addPermute(dd_node_ptr::get_cudd_manager(), mdp_->get_value_function().get(), mdp_->get_unprimed_to_primed_permutation()));
		record_values_ = compute_action_cost(primed_value_function, mdp_->get_transitions().begin()->second);
		
		for (SymbolicMDP::transitions_map_t::const_iterator Itr = ++(mdp_->get_transitions().begin()) ; Itr != mdp_->get_transitions().end() ; ++Itr)
		{
			dd_node_ptr action_cost = compute_action_cost(primed_value_function, Itr->second);
			record_values_ = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addMaximum, record_values_.get(), action_cost.get()));
		}
		
		compute_bellman_error(record_values_, mdp_->get_value_function());
		mdp_->_value_function_().copy(record_values_);
    }
}


bool SymbolicGCO::has_converged()
{
	if (probability_converged_ && (bellman_error_ < epsilon_))
	{
		dd_node_ptr primed_value_function(Cudd_addPermute(dd_node_ptr::get_cudd_manager(), mdp_->get_value_function().get(), mdp_->get_unprimed_to_primed_permutation()));
		compute_cost_policy(mdp_->_policy_(), mdp_->_value_function_(), primed_value_function, mdp_->get_transitions());
		return true;
	}
	else
		return false;
}


dd_node_ptr SymbolicGCO::compute_action_probability(const dd_node_ptr& primed_probability_function, const SymbolicTransition& transitions) const
{
	dd_node_ptr action_probability(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, primed_probability_function.get(), transitions.get_probabilities().get()));
	action_probability = dd_node_ptr(Cudd_addExistAbstract(dd_node_ptr::get_cudd_manager(), action_probability.get(), mdp_->get_primed_variables_add_cube().get()));
	action_probability = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, action_probability.get(), transitions.get_value_mask().get()));
	return action_probability;
}


dd_node_ptr SymbolicGCO::compute_action_cost(const dd_node_ptr& primed_value_function, const SymbolicTransition& transitions) const
{
	dd_node_ptr action_probability_function(Cudd_addPermute(dd_node_ptr::get_cudd_manager(), mdp_->get_probability_function().get(), mdp_->get_unprimed_to_primed_permutation()));
	action_probability_function = compute_action_probability(action_probability_function, transitions);
	action_probability_function = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, action_probability_function.get(), transitions.get_value_mask().get()));
	dd_node_ptr action_greedy_states(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addMinus, action_probability_function.get(), mdp_->get_probability_function().get()));
	action_greedy_states = dd_node_ptr(Cudd_addBddInterval(dd_node_ptr::get_cudd_manager(), action_greedy_states.get(), -epsilon_, epsilon_));
	dd_node_ptr action_mask(Cudd_Not(action_greedy_states.get()));
	action_mask = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), action_mask.get()));
	dd_node_ptr minus_infinity_dd(Cudd_ReadMinusInfinity(dd_node_ptr::get_cudd_manager()));
	action_mask = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, action_mask.get(), minus_infinity_dd.get()));
	action_greedy_states = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), action_greedy_states.get()));

	dd_node_ptr action_cost(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, primed_value_function.get(), transitions.get_rewards().get()));
	action_cost = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, action_cost.get(), post_probability_function_.get()));
	action_cost = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, action_cost.get(), transitions.get_probabilities().get()));
	action_cost = dd_node_ptr(Cudd_addExistAbstract(dd_node_ptr::get_cudd_manager(), action_cost.get(), mdp_->get_primed_variables_add_cube().get()));
	action_cost = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addDivide, action_cost.get(), pre_probability_function_.get()));
	action_cost = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, action_cost.get(), action_greedy_states.get()));
	action_cost = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, action_cost.get(), action_mask.get()));
	
	return action_cost;
}


void SymbolicGCO::compute_cost_policy(dd_node_ptr& policy, dd_node_ptr& value_function, const dd_node_ptr& primed_value_function, const SymbolicMDP::transitions_map_t& transitions) const
{
	policy = dd_node_ptr(Cudd_ReadZero(dd_node_ptr::get_cudd_manager()));
	
	if (mdp_->get_index_to_action().empty())
		return;
	
	dd_node_ptr not_states(Cudd_Not(restricted_states_.get()));
	
	dd_node_ptr minus_infinity_dd(Cudd_ReadMinusInfinity(dd_node_ptr::get_cudd_manager()));
	value_function = compute_action_cost(primed_value_function, transitions.find(mdp_->get_index_to_action().begin()->second)->second);
	policy = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addOneZeroMaximum, value_function.get(), minus_infinity_dd.get()));
	policy = dd_node_ptr(Cudd_addBddPattern(dd_node_ptr::get_cudd_manager(), policy.get()));
	dd_node_ptr action_cmpl_mask(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), transitions.find(mdp_->get_index_to_action().begin()->second)->second.get_policy_mask().get(), not_states.get()));
	policy = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), policy.get(), action_cmpl_mask.get()));
	policy = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), policy.get()));
	dd_node_ptr action_dd(Cudd_addConst(dd_node_ptr::get_cudd_manager(), mdp_->get_index_to_action().begin()->first));
	policy = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, policy.get(), action_dd.get()));
	
	for (std::map<CUDD_VALUE_TYPE, const Action*>::const_iterator Itr = ++(mdp_->get_index_to_action().begin()) ; Itr != mdp_->get_index_to_action().end() ; ++Itr)
	{
		dd_node_ptr action_value = compute_action_cost(primed_value_function, transitions.find(Itr->second)->second);
		dd_node_ptr vf_before_max = value_function;
		value_function = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addMaximum, vf_before_max.get(), action_value.get()));
		dd_node_ptr improve_set_dd(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addOneZeroMaximum, action_value.get(), vf_before_max.get()));
		improve_set_dd = dd_node_ptr(Cudd_addBddPattern(dd_node_ptr::get_cudd_manager(), improve_set_dd.get()));
		action_cmpl_mask = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), transitions.find(Itr->second)->second.get_policy_mask().get(), not_states.get()));
		improve_set_dd = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), improve_set_dd.get(), action_cmpl_mask.get()));
		improve_set_dd = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), improve_set_dd.get()));
		dd_node_ptr cmpl_dd(Cudd_addCmpl(dd_node_ptr::get_cudd_manager(), improve_set_dd.get()));
		
		action_dd = dd_node_ptr(Cudd_addConst(dd_node_ptr::get_cudd_manager(), Itr->first));
		improve_set_dd = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, improve_set_dd.get(), action_dd.get()));
		cmpl_dd = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, cmpl_dd.get(), policy.get()));
		policy = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, improve_set_dd.get(), cmpl_dd.get()));
	}
}

