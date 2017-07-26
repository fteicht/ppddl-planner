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

#include "hadd.h"
#include <typeinfo>
#include <algorithm>
#include <cmath>

// CLASS HeuristicHadd

HeuristicHadd::HeuristicHadd(double discount_factor, const Problem& pb)
try : BaseHeuristic(discount_factor), problem_(pb)
{
	true_atoms_weights_.resize(Atom::number_of_atoms());
	false_atoms_weights_.resize(Atom::number_of_atoms());
}
catch (BaseException & error)
{
	error.push_function_backtrace("HeuristicHadd::HeuristicHadd");
	throw;
}


double HeuristicHadd::value(const PddlState& st)
{
	std::fill(true_atoms_weights_.begin(), true_atoms_weights_.end(), std::numeric_limits<unsigned int>::max());
	std::fill(false_atoms_weights_.begin(), false_atoms_weights_.end(), 0);
	
	for (AtomSet::const_iterator Iat = st.atoms().begin() ; Iat != st.atoms().end() ; ++Iat)
	{
		true_atoms_weights_[(*Iat)->index()] = 0;
		false_atoms_weights_[(*Iat)->index()] = std::numeric_limits<unsigned int>::max();
	}
	
	bool continue_flag = true;
	
	while (continue_flag)
	{
		continue_flag = false;
		
		for (ActionSet::const_iterator Ia = problem_.actions().begin() ; Ia != problem_.actions().end() ; ++Ia)
		{
			//double weight = relaxed_hold((*Ia)->precondition());
			double weight = (*Ia)->precondition().hadd_holds(true_atoms_weights_, false_atoms_weights_);
			
			if (weight != std::numeric_limits<unsigned int>::max())
				continue_flag = ((*Ia)->effect().hadd_relaxation(true_atoms_weights_, false_atoms_weights_, weight + 1)) || continue_flag;
				//continue_flag = relaxed_effect((*Ia)->effect(), weight + 1) || continue_flag;
		}
	}
	
	//double weight = relaxed_hold(problem_.goal());
	double weight = problem_.goal().hadd_holds(true_atoms_weights_, false_atoms_weights_);
	
	if (discount_factor_ < 1.0)
	{
		if (weight != std::numeric_limits<unsigned int>::max())
			return (weight < 2)?(0.0):((1.0 - std::pow(discount_factor_, weight - 1))/(discount_factor_ - 1.0));
		else
			return (-1.0 / (1.0 - discount_factor_));
	}
	else
	{
		if (weight != std::numeric_limits<unsigned int>::max())
			return (weight < 2)?(0.0):(1 - weight);
		else
			return (-std::numeric_limits<double>::max());
	}
}


unsigned int HeuristicHadd::relaxed_hold(const StateFormula& formula, bool negate) const
{
	if (formula.tautology())
		return 0;

	if (formula.contradiction())
		return std::numeric_limits<unsigned int>::max();

	const Atom* atom_formula = dynamic_cast<const Atom*>(&formula);

	if (atom_formula)
	{
		if (negate)
			return false_atoms_weights_[atom_formula->index()];
		else
			return true_atoms_weights_[atom_formula->index()];
	}

	const Negation* negation_formula = dynamic_cast<const Negation*>(&formula);

	if (negation_formula)
		return relaxed_hold(negation_formula->negand(), !negate);

	const Conjunction* conjunction_formula = dynamic_cast<const Conjunction*>(&formula);

	if (conjunction_formula)
	{
		if (negate)
		{
			unsigned int weight = std::numeric_limits<unsigned int>::max();
			
			for (FormulaList::const_iterator If = conjunction_formula->conjuncts().begin() ; If != conjunction_formula->conjuncts().end() ; ++If)
				weight = std::min(weight, relaxed_hold(**If, negate));
			
			return weight;
		}
		else
		{
			unsigned int weight = 0;
			
			for (FormulaList::const_iterator If = conjunction_formula->conjuncts().begin() ; If != conjunction_formula->conjuncts().end() ; ++If)
			{
				unsigned int wtp = relaxed_hold(**If, negate);
				
				if (wtp != std::numeric_limits<unsigned int>::max())
					weight += wtp;
				else
					return std::numeric_limits<unsigned int>::max();
			}

			return weight;
		}
	}

	const Disjunction* disjunction_formula = dynamic_cast<const Disjunction*>(&formula);

	if (disjunction_formula)
	{
		if (negate)
		{
			unsigned int weight = 0;
			
			for (FormulaList::const_iterator If = disjunction_formula->disjuncts().begin() ; If != disjunction_formula->disjuncts().end() ; ++If)
			{
				unsigned int wtp = relaxed_hold(**If, negate);
				
				if (wtp != std::numeric_limits<unsigned int>::max())
					weight += wtp;
				else
					return std::numeric_limits<unsigned int>::max();
			}

			return weight;
		}
		else
		{
			unsigned int weight = std::numeric_limits<unsigned int>::max();
			
			for (FormulaList::const_iterator If = disjunction_formula->disjuncts().begin() ; If != disjunction_formula->disjuncts().end() ; ++If)
				weight = std::min(weight, relaxed_hold(**If, negate));	

			return weight;
		}
	}

	// No other types of (instantiated) state formula exist
	throw BaseException(std::string("unexpected state formula ") + typeid(formula).name(), "HeuristicHadd::relaxed_hold");
}


bool HeuristicHadd::relaxed_effect(const Effect& effect, unsigned int weight)
{
	if (effect.empty())
		return false;

	const AddEffect* add_effect = dynamic_cast<const AddEffect*>(&effect);

	if (add_effect)
	{
		if (true_atoms_weights_[add_effect->atom().index()] > weight)
		{
			true_atoms_weights_[add_effect->atom().index()] = weight;
			return true;
		}
		else
			return false;
	}

	const DeleteEffect* delete_effect = dynamic_cast<const DeleteEffect*>(&effect);

	if (delete_effect)
	{
		if (false_atoms_weights_[delete_effect->atom().index()] > weight)
		{
			false_atoms_weights_[delete_effect->atom().index()] = weight;
			return true;
		}
		else
			return false;
	}

	const UpdateEffect* update_effect = dynamic_cast<const UpdateEffect*>(&effect);

	if (update_effect)
		return false;

	const ConjunctiveEffect* conjunctive_effect = dynamic_cast<const ConjunctiveEffect*>(&effect);

	if (conjunctive_effect)
	{
		bool flag = false;
		
		for (EffectList::const_iterator Ie = conjunctive_effect->conjuncts().begin() ; Ie != conjunctive_effect->conjuncts().end() ; ++Ie)
			flag = relaxed_effect(**Ie, weight) || flag;

		return flag;
	}

	const ConditionalEffect* conditional_effect = dynamic_cast<const ConditionalEffect*>(&effect);

	if (conditional_effect)
	{
		bool flag = false;
		
		if (relaxed_hold(conditional_effect->condition()))
			flag = relaxed_effect(conditional_effect->effect(), weight) || flag;

		return flag;
	}

	const ProbabilisticEffect* probabilistic_effect = dynamic_cast<const ProbabilisticEffect*>(&effect);

	if (probabilistic_effect)
	{
		size_t nb_outcomes = probabilistic_effect->size();
		bool flag = false;

		for (unsigned int cnt = 0 ; cnt < nb_outcomes ; cnt++)
			flag = relaxed_effect(probabilistic_effect->effect(cnt), weight) || flag;

		return flag;
	}

	// No other types of (instantiated) effect exist
	throw BaseException(std::string("unexpected effect ") + typeid(effect).name(), "HeuristicHadd::relaxed_effect");
}




// CLASS GraphHeuristicHadd

template <class Tstate>
GraphHeuristicHadd<Tstate>::GraphHeuristicHadd(GraphMDP<Tstate>& mdp, double discount_factor, const Problem& pb)
try : BaseHeuristic(discount_factor), GraphHeuristic<Tstate>(mdp, discount_factor), HeuristicHadd(discount_factor, pb)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphHeuristicHadd::GraphHeuristicHadd");
	throw;
}


template <class Tstate>
void GraphHeuristicHadd<Tstate>::compute(Tstate& st)
{
	st.set_value(value(st.get_pddl_state()));
}


// Explicit template instantiations
INSTANTIATE_STATE_TEMPLATE(GraphHeuristicHadd)




// CLASS SymbolicHeuristicHadd

SymbolicHeuristicHadd::SymbolicHeuristicHadd(SymbolicMDP& mdp, double discount_factor, const Problem& pb)
try : BaseHeuristic(discount_factor), SymbolicHeuristic(mdp, discount_factor), HeuristicHadd(discount_factor, pb)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("SymbolicHeuristicHadd::SymbolicHeuristicHadd");
	throw;
}


void SymbolicHeuristicHadd::compute(const dd_node_ptr& states)
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
