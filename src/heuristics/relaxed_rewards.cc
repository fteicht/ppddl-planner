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

#include <typeinfo>
#include <limits>
#include <cmath>

#include "relaxed_rewards.h"
#include "commons/base_exceptions.h"


// CLASS RelaxedRewards

RelaxedRewards::RelaxedRewards(double discount_factor, const Problem& pb)
try : BaseHeuristic(discount_factor), problem_(pb)
{
	const Function* reward_function = problem_.domain().functions().find_function("reward");
	
	if (!reward_function)
		throw IncompatibilityException("domain contains no reward function");
	
	const Function* total_time_function = problem_.domain().functions().find_function("total-time");
	
	if (total_time_function)
		values_.insert(std::make_pair(Fluent::fluent(*total_time_function), Rational(0)));
	
	const Function* goal_achieved_function = problem_.domain().functions().find_function("goal-achieved");
	
	if (goal_achieved_function)
		values_.insert(std::make_pair(Fluent::fluent(*goal_achieved_function), Rational(0)));
	
	reward_fluent_= values_.insert(std::make_pair(Fluent::fluent(*reward_function), Rational(0))).first;
}
catch (BaseException & error)
{
	error.push_function_backtrace("RelaxedRewards::RelaxedRewards");
	throw;
}


bool RelaxedRewards::update_rewards()
{
	AtomSet true_prd = true_predicates_;
	AtomSet false_prd = false_predicates_;
	
	for (ActionSet::const_iterator Ia = problem_.actions().begin() ; Ia != problem_.actions().end() ; ++Ia)
	{
		if (relaxed_hold((*Ia)->precondition()))
			relaxed_effect((*Ia)->effect(), true_prd, false_prd);
	}
	
	if ((true_prd == true_predicates_) && (false_prd == false_predicates_))
		return false;
	else
	{
		true_predicates_ = true_prd;
		false_predicates_ = false_prd;
		return true;
	}
}


void RelaxedRewards::update_goal_reward()
{
	if ((problem_.goal_reward()) && relaxed_hold(problem_.goal()))
	{
		reward_fluent_->second = Rational(0);
		problem_.goal_reward()->affect(values_);
		double rwd = reward_fluent_->second.double_value();
		
		if (first_reward_)
		{
			min_level_reward_ = rwd;
			max_level_reward_ = rwd;
		}
		else
		{
			min_level_reward_ += rwd;
			max_level_reward_ += rwd;
		}
	}
}


double RelaxedRewards::value(const PddlState& st)
{
	true_predicates_ = st.atoms();
	false_predicates_ = st.atoms();
	first_reward_ = true;
	first_level_ = true;
	
	while (update_rewards())
	{
		update_goal_reward();
		first_reward_ = true;
		
		if (first_level_)
		{
			min_reward_ = min_level_reward_;
			max_reward_ = max_level_reward_;
			first_level_ = false;
		}
		else
		{
			min_reward_ = std::min(min_reward_, min_level_reward_);
			max_reward_ = std::max(max_reward_, max_level_reward_);
		}
	}
	
	if (discount_factor_ < 1.0)
		return (max_reward_ / (1.0 - discount_factor_));
	else
		return ((max_reward_ > 0.0)?(std::numeric_limits<double>::max()):(-std::numeric_limits<double>::max()));
}


bool RelaxedRewards::relaxed_hold(const StateFormula& formula, bool negate) const
{
	if (formula.tautology())
		return true;
	
	if (formula.contradiction())
		return false;
	
	const Atom* atom_formula = dynamic_cast<const Atom*>(&formula);
	
	if (atom_formula)
	{
		if (negate)
			return (false_predicates_.find(atom_formula) == false_predicates_.end());
		else
			return (true_predicates_.find(atom_formula) != true_predicates_.end());
	}
	
	const Negation* negation_formula = dynamic_cast<const Negation*>(&formula);
	
	if (negation_formula)
		return relaxed_hold(negation_formula->negand(), !negate);
	
	const Conjunction* conjunction_formula = dynamic_cast<const Conjunction*>(&formula);
	
	if (conjunction_formula)
	{
		if (negate)
		{
			for (FormulaList::const_iterator If = conjunction_formula->conjuncts().begin() ; If != conjunction_formula->conjuncts().end() ; ++If)
			{
				if (relaxed_hold(**If, negate))
					return true;
			}
			
			return false;
		}
		else
		{
			for (FormulaList::const_iterator If = conjunction_formula->conjuncts().begin() ; If != conjunction_formula->conjuncts().end() ; ++If)
			{
				if (!relaxed_hold(**If, negate))
					return false;
			}
			
			return true;
		}
	}
	
	const Disjunction* disjunction_formula = dynamic_cast<const Disjunction*>(&formula);
	
	if (disjunction_formula)
	{
		if (negate)
		{
			for (FormulaList::const_iterator If = disjunction_formula->disjuncts().begin() ; If != disjunction_formula->disjuncts().end() ; ++If)
			{
				if (!relaxed_hold(**If, negate))
					return false;
			}
			
			return true;
		}
		else
		{
			for (FormulaList::const_iterator If = disjunction_formula->disjuncts().begin() ; If != disjunction_formula->disjuncts().end() ; ++If)
			{
				if (relaxed_hold(**If, negate))
					return true;
			}
			
			return false;
		}
	}
	
	// No other types of (instantiated) state formula exist
	throw BaseException(std::string("unexpected state formula ") + typeid(formula).name(), "RelaxedRewards::relaxed_hold");
}


void RelaxedRewards::relaxed_effect(const Effect& effect, AtomSet& true_prd, AtomSet& false_prd)
{
	if (effect.empty())
		return;
	
	const AddEffect* add_effect = dynamic_cast<const AddEffect*>(&effect);
	
	if (add_effect)
	{
		true_prd.insert(&(add_effect->atom()));
		return;
	}
	
	const DeleteEffect* delete_effect = dynamic_cast<const DeleteEffect*>(&effect);
	
	if (delete_effect)
	{
		false_prd.erase(&(delete_effect->atom()));
		return;
	}
	
	const UpdateEffect* update_effect = dynamic_cast<const UpdateEffect*>(&effect);
	
	if (update_effect)
	{
		reward_fluent_->second = Rational(0);
		update_effect->update().affect(values_);
		double rwd = reward_fluent_->second.double_value();
		
		if (first_reward_)
		{
			min_level_reward_ = rwd;
			max_level_reward_ = rwd;
			first_reward_ = false;
		}
		else
		{
			min_level_reward_ = std::min(min_level_reward_, rwd);
			max_level_reward_ = std::max(max_level_reward_, rwd);
		}
		
		return;
	}
	
	const ConjunctiveEffect* conjunctive_effect = dynamic_cast<const ConjunctiveEffect*>(&effect);
	
	if (conjunctive_effect)
	{
		for (EffectList::const_iterator Ie = conjunctive_effect->conjuncts().begin() ; Ie != conjunctive_effect->conjuncts().end() ; ++Ie)
			relaxed_effect(**Ie, true_prd, false_prd);
		
		return;
	}
	
	const ConditionalEffect* conditional_effect = dynamic_cast<const ConditionalEffect*>(&effect);
	
	if (conditional_effect)
	{
		if (relaxed_hold(conditional_effect->condition()))
			relaxed_effect(conditional_effect->effect(), true_prd, false_prd);
		
		return;
	}
	
	const ProbabilisticEffect* probabilistic_effect = dynamic_cast<const ProbabilisticEffect*>(&effect);
	
	if (probabilistic_effect)
	{
		size_t nb_outcomes = probabilistic_effect->size();
		
		for (unsigned int cnt = 0 ; cnt < nb_outcomes ; cnt++)
			relaxed_effect(probabilistic_effect->effect(cnt), true_prd, false_prd);
		
		return;
	}
	
	// No other types of (instantiated) effect exist
	throw BaseException(std::string("unexpected effect ") + typeid(effect).name(), "RelaxedRewards::relaxed_effect");
}




// CLASS GraphRelaxedRewards

template <class Tstate>
GraphRelaxedRewards<Tstate>::GraphRelaxedRewards(GraphMDP<Tstate>& mdp, double discount_factor, const Problem& pb)
try : BaseHeuristic(discount_factor), GraphHeuristic<Tstate>(mdp, discount_factor), RelaxedRewards(discount_factor, pb)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphRelaxedRewards::GraphRelaxedRewards");
	throw;
}


template <class Tstate>
void GraphRelaxedRewards<Tstate>::compute(Tstate& st)
{
	st.set_value(value(st.get_pddl_state()));
}


// Explicit template instantiations
INSTANTIATE_STATE_TEMPLATE(GraphRelaxedRewards)




// CLASS SymbolicRelaxedStepDistance

SymbolicRelaxedRewards::SymbolicRelaxedRewards(SymbolicMDP& mdp, double discount_factor, const Problem& pb)
try : BaseHeuristic(discount_factor), SymbolicHeuristic(mdp, discount_factor), RelaxedRewards(discount_factor, pb)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("SymbolicRelaxedRewards::SymbolicRelaxedRewards");
	throw;
}


void SymbolicRelaxedRewards::compute(const dd_node_ptr& states)
{
	dd_node_ptr heuristic_values(Cudd_ReadZero(dd_node_ptr::get_cudd_manager()));
	
	std::list<PddlState> pddl_states;
	mdp_.decompose(states, pddl_states);
	
	for (std::list<PddlState>::const_iterator Ist = pddl_states.begin() ; Ist != pddl_states.end() ; ++Ist)
	{
		dd_node_ptr valdd(Cudd_addConst(dd_node_ptr::get_cudd_manager(), value(*Ist)));
		dd_node_ptr cudd_state = mdp_.get_state(*Ist);
		cudd_state = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), cudd_state.get()));
		cudd_state = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, cudd_state.get(), valdd.get()));
		heuristic_values = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, heuristic_values.get(), cudd_state.get()));
	}
	
	dd_node_ptr states_cmpl = dd_node_ptr(Cudd_Not(states.get()));
	states_cmpl = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), states_cmpl.get()));
	mdp_._value_function_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, mdp_.get_value_function().get(), states_cmpl.get()));
	mdp_._value_function_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, mdp_.get_value_function().get(), heuristic_values.get()));
}
