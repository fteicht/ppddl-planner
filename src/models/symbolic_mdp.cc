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

#include <sstream>
#include <algorithm>
#include <unistd.h>
#include <cstdlib>
#include <cstdio>
#include <mdpsim/formulas.h>

const StateFormula& state_formula_false = StateFormula::FALSE; // to avoid clashes with CUDD's FALSE

#include "symbolic_mdp.h"
#include "commons/base_exceptions.h"

#include <cuddInt.h>

// CLASS SymbolicTransition

SymbolicTransition::SymbolicTransition(const dd_node_ptr& policy_mask, const dd_node_ptr& value_mask, const dd_node_ptr& probabilities, const dd_node_ptr& rewards)
{
	policy_mask_.copy(policy_mask);
	value_mask_.copy(value_mask);
	probabilities_.copy(probabilities);
	rewards_.copy(rewards);
}


void SymbolicTransition::copy(const SymbolicTransition& other)
{
	this->policy_mask_.copy(other.policy_mask_);
	this->value_mask_.copy(other.value_mask_);
	this->probabilities_.copy(other.probabilities_);
	this->rewards_.copy(other.rewards_);
}


SymbolicTransition::operator bool () const
{
	return ((policy_mask_) && (value_mask_) && (probabilities_) && (rewards_));
}


bool SymbolicTransition::operator!() const
{
	return ((!policy_mask_) || (!value_mask_) || (!probabilities_) || (!rewards_));
}


// CLASS SymbolicMDP

SymbolicMDP::SymbolicMDP(const TransitionsBlackbox& tb)
try : BaseMDP(tb), unprimed_to_primed_permutation_(NULL), primed_to_unprimed_permutation_(NULL)
{
	dd_node_ptr::initialize_cudd_manager();
	dd_node_ptr::set_cudd_reordering_method(CUDD_REORDER_NONE);//SIFT);
	states_ = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	unexpanded_states_ = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	goal_states_ = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	policy_ = dd_node_ptr(Cudd_ReadZero(dd_node_ptr::get_cudd_manager()));
	value_function_ = dd_node_ptr(Cudd_ReadZero(dd_node_ptr::get_cudd_manager()));
	probability_function_ = dd_node_ptr(Cudd_ReadZero(dd_node_ptr::get_cudd_manager()));
	
	ScopedConnectionDiagram::set_mdp(this);
	extern int verbosity; // defined in main_planner[test|client].cc
	
	if (verbosity >= 2)
	{
		Cudd_EnableReorderingReporting(dd_node_ptr::get_cudd_manager());;
		Cudd_PrintInfo(dd_node_ptr::get_cudd_manager(), stdout);
	}
}
catch (BaseException& error)
{
	error.push_function_backtrace("SymbolicMDP::SymbolicMDP");
	throw;
}


SymbolicMDP::~SymbolicMDP()
{
	try
	{
		states_.clear();
		unexpanded_states_.clear();
		goal_states_.clear();
		transitions_.clear();
		policy_.clear();
		value_function_.clear();
		probability_function_.clear();
		states_record_.clear();
		state_blackbox_.clear();
		unprimed_variables_bdd_cube_.clear();
		unprimed_variables_add_cube_.clear();
		primed_variables_bdd_cube_.clear();
		primed_variables_add_cube_.clear();
		
		dd_node_ptr::shutdown_cudd_manager();
	}
	catch (BaseException& error)
	{
		std::cerr << error.what() << std::endl;
	}
	
	if (unprimed_to_primed_permutation_)
		delete[] unprimed_to_primed_permutation_;
	
	if (primed_to_unprimed_permutation_)
		delete[] primed_to_unprimed_permutation_;
	
	if (ScopedConnectionDiagram::get_mdp() == this)
		ScopedConnectionDiagram::set_mdp(NULL);
}


void SymbolicMDP::transition_blackbox(const Action& action, double probability, double reward, PddlState* outcome, bool goal)
{
	std::pair<transitions_map_t::iterator, bool> Itr = transitions_.insert(std::make_pair(&action, SymbolicTransition()));
	
	if (Itr.second) // new action
	{
		Itr.first->second._policy_mask_() = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
		Itr.first->second._value_mask_() = dd_node_ptr(Cudd_ReadMinusInfinity(dd_node_ptr::get_cudd_manager()));
		Itr.first->second._probabilities_() = dd_node_ptr(Cudd_ReadZero(dd_node_ptr::get_cudd_manager()));
		Itr.first->second._rewards_() = dd_node_ptr(Cudd_ReadZero(dd_node_ptr::get_cudd_manager()));
		index_to_action_.insert(std::make_pair((index_to_action_.size()) + 1, &action));
	}
	
	dd_node_ptr dd_outcome = get_and_add_cudd_state(*outcome, goal);
	delete outcome;
	
	SymbolicTransition& transition = Itr.first->second;
	transition._policy_mask_() = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), transition.get_policy_mask().get(), state_blackbox_.get()));
	
	dd_node_ptr state_blackbox_cmpl(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), state_blackbox_.get()));
	state_blackbox_cmpl = dd_node_ptr(Cudd_addCmpl(dd_node_ptr::get_cudd_manager(), state_blackbox_cmpl.get()));
	transition._value_mask_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, transition.get_value_mask().get(), state_blackbox_cmpl.get()));
	
	dd_node_ptr new_transition = dd_node_ptr(Cudd_bddPermute(dd_node_ptr::get_cudd_manager(), dd_outcome.get(), unprimed_to_primed_permutation_));
	new_transition = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), new_transition.get(), state_blackbox_.get()));
	new_transition = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), new_transition.get()));
	dd_node_ptr dd_probability(Cudd_addConst(dd_node_ptr::get_cudd_manager(), probability));
	dd_node_ptr new_transition_probability = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, new_transition.get(), dd_probability.get()));
	transition._probabilities_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, transition.get_probabilities().get(), new_transition_probability.get()));
	
	dd_node_ptr dd_reward(Cudd_addConst(dd_node_ptr::get_cudd_manager(), reward));
	dd_node_ptr new_transition_reward = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, new_transition.get(), dd_reward.get()));
	transition._rewards_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, transition.get_rewards().get(), new_transition_reward.get()));
}


std::pair<dd_node_ptr, double> SymbolicMDP::random_transition(const dd_node_ptr& st, const Action& action)
{
	dd_node_ptr st_bis = expanded(st);
	
	if (st_bis.get() == Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager())) // pick from PDDL model
	{
		double reward;
		bool goal;
		PddlState income_st;
		PddlState outcome_st;
		get_pddl_state(st, income_st);
		tb_.random_transition(income_st, action, outcome_st, reward, goal);
		return std::make_pair(get_and_add_cudd_state(outcome_st, goal), reward);
	}
	else if (st_bis.get() == st.get())
	{
		transitions_map_t::const_iterator Itr = transitions_.find(&action);
		
		if (Itr == transitions_.end())
			throw BaseException("unknown action", "SymbolicMDP::random_transition");
		
		st_bis = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), st_bis.get()));
		dd_node_ptr state_transitions(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, st_bis.get(), Itr->second.get_probabilities().get()));
		state_transitions = dd_node_ptr(Cudd_addExistAbstract(dd_node_ptr::get_cudd_manager(), state_transitions.get(), unprimed_variables_add_cube_.get()));
		state_transitions = dd_node_ptr(Cudd_addPermute(dd_node_ptr::get_cudd_manager(), state_transitions.get(), primed_to_unprimed_permutation_));
		dd_node_ptr next_st = pick_state(state_transitions);
		dd_node_ptr reward_dd(Cudd_bddPermute(dd_node_ptr::get_cudd_manager(), next_st.get(), unprimed_to_primed_permutation_));
		reward_dd = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), reward_dd.get()));
		reward_dd = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, st_bis.get(), reward_dd.get()));
		DdNode* reward_dd_node = Cudd_addEvalConst(dd_node_ptr::get_cudd_manager(), reward_dd.get(), Itr->second.get_rewards().get());
		
		if (reward_dd.get() == DD_NON_CONSTANT)
			throw BaseException("ill-formed next state", "SymbolicMDP::random_transition");
		else
		{
			reward_dd = dd_node_ptr(reward_dd_node);
			return std::make_pair(next_st, Cudd_V(reward_dd.get()));
		}
	}
	else
		throw BaseException("input is a set of states (should be a single state)", "SymbolicMDP::random_transition");
}


dd_node_ptr SymbolicMDP::pick_state(const dd_node_ptr& diagram) const
{
	int* states_cube;
	CUDD_VALUE_TYPE probability;
	DdGen* gen;
	std::list<PddlState> pddl_states;
	std::list<double> states_probabilities;
	
	if (dd_node_ptr::get_cudd_reordering_method() != CUDD_REORDER_NONE)
		Cudd_AutodynDisable(dd_node_ptr::get_cudd_manager());
	
	Cudd_ForeachCube(dd_node_ptr::get_cudd_manager(), diagram.get(), gen, states_cube, probability)
	{
		unsigned int size_record = pddl_states.size();
		push_pddl_states(states_cube, pddl_states);
		unsigned int new_size = pddl_states.size();
		
		for (unsigned int cnt = 0 ; cnt < (new_size - size_record) ; cnt++)
			states_probabilities.push_back(probability);
	}
	
	if (dd_node_ptr::get_cudd_reordering_method() != CUDD_REORDER_NONE)
		Cudd_AutodynEnable(dd_node_ptr::get_cudd_manager(), CUDD_REORDER_SAME);
	
	double prob = std::rand() / static_cast<double>(RAND_MAX);
	double cumulative_prob = 0.0;
	std::list<PddlState>::const_iterator Ist = pddl_states.begin();
	
	for (std::list<double>::const_iterator Iprob = states_probabilities.begin() ; Iprob != states_probabilities.end() ; ++Iprob)
	{
		cumulative_prob += *Iprob;
		
		if (prob <= cumulative_prob)
			return get_cudd_state(*Ist);
		
		++Ist;
	}
	
	throw BaseException("this line should never be reached", "SymbolicMDP::pick_state");
}


dd_node_ptr SymbolicMDP::expand(const dd_node_ptr& states)
{
	states_record_.copy(states_);
	std::list<PddlState> pddl_states;
	decompose(states, pddl_states);
	
	for (std::list<PddlState>::const_iterator Ips = pddl_states.begin() ; Ips != pddl_states.end() ; ++Ips)
	{
		state_blackbox_ = get_cudd_state(*Ips);
		tb_.transitions(*Ips, *this);
	}
	
	unexpanded_states_ = dd_node_ptr(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), unexpanded_states_.get(), states.get()));
	return dd_node_ptr(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), states_record_.get(), states_.get()));
}


void SymbolicMDP::decompose(const dd_node_ptr& cudd_states, std::list<PddlState>& pddl_states) const
{
	int* states_cube;
	CUDD_VALUE_TYPE value;
	DdGen* gen;
	pddl_states.clear();
	
	if (dd_node_ptr::get_cudd_reordering_method() != CUDD_REORDER_NONE)
		Cudd_AutodynDisable(dd_node_ptr::get_cudd_manager());
	
	Cudd_ForeachCube(dd_node_ptr::get_cudd_manager(), cudd_states.get(), gen, states_cube, value)
	{
		push_pddl_states(states_cube, pddl_states);
	}
	
	if (dd_node_ptr::get_cudd_reordering_method() != CUDD_REORDER_NONE)
		Cudd_AutodynEnable(dd_node_ptr::get_cudd_manager(), CUDD_REORDER_SAME);
}


void SymbolicMDP::push_pddl_states(int* states_cube, std::list<PddlState>& pddl_states) const
{
	std::list<PddlState>::iterator Isublist_begin = pddl_states.insert(pddl_states.end(), PddlState());
	unsigned int sublist_size = 1;
	
	for (std::map<const Atom*, int>::const_iterator Imp = atom_to_index_.begin() ; Imp != atom_to_index_.end() ; ++Imp)
	{
		if (states_cube[Imp->second] == 1)
		{
			for (std::list<PddlState>::iterator Ips = Isublist_begin ; Ips != pddl_states.end() ; ++Ips)
				Ips->_atoms_().insert(Imp->first);
		}
		else if (states_cube[Imp->second] != 0)
		{
			std::list<PddlState>::iterator Ips = Isublist_begin;
			unsigned int sublist_size_record = sublist_size;
			
			for (unsigned int cnt = 0 ; cnt < sublist_size_record ; cnt++)
			{
				pddl_states.push_back(PddlState());
				pddl_states.back()._atoms_() = Ips->atoms();
				pddl_states.back()._atoms_().insert(Imp->first);
				Ips++;
				sublist_size++;
			}
		}
	}
}


void SymbolicMDP::get_pddl_state(const dd_node_ptr& cudd_state, PddlState& pddl_state) const
{
	dd_node_ptr essential_variables_cube(Cudd_FindEssential(dd_node_ptr::get_cudd_manager(), cudd_state.get()));
	int* positional_array = new int[Cudd_ReadSize(dd_node_ptr::get_cudd_manager())];
	Cudd_BddToCubeArray(dd_node_ptr::get_cudd_manager(), essential_variables_cube.get(), positional_array);
	
	for (std::map<const Atom*, int>::const_iterator Imp = atom_to_index_.begin() ; Imp != atom_to_index_.end() ; ++Imp)
	{
		if (positional_array[Imp->second] == 1)
			pddl_state._atoms_().insert(Imp->first);
	}
	
	delete[] positional_array;
}


bool SymbolicMDP::_cudd_state_(const PddlState& pddl_state, dd_node_ptr& cudd_state) const
{
	cudd_state = dd_node_ptr(Cudd_ReadOne(dd_node_ptr::get_cudd_manager()));
	unsigned int nb_of_atoms_found = 0;
	
	for (std::map<const Atom*, int>::const_iterator Imp = atom_to_index_.begin() ; Imp != atom_to_index_.end() ; ++Imp)
	{
		dd_node_ptr var(Cudd_bddIthVar(dd_node_ptr::get_cudd_manager(), Imp->second), false);
		
		if (pddl_state.atoms().find(Imp->first) != pddl_state.atoms().end())
		{
			cudd_state = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), cudd_state.get(), var.get()));
			nb_of_atoms_found++;
		}
		else
			cudd_state = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), cudd_state.get(), Cudd_Not(var.get())));
	}
	
	return (nb_of_atoms_found == pddl_state.atoms().size());
}


dd_node_ptr SymbolicMDP::get_cudd_state(const PddlState& st) const
{
	dd_node_ptr cudd_state;
	
	if (!_cudd_state_(st, cudd_state))
		throw BaseException("unknown atoms", "SymbolicMDP::get_cudd_state");
	
	return cudd_state;
}


dd_node_ptr SymbolicMDP::get_and_add_cudd_state(const PddlState& st, bool goal)
{
	dd_node_ptr cudd_state(Cudd_ReadOne(dd_node_ptr::get_cudd_manager()));
	AtomSet atoms_found;
	
	for (std::map<const Atom*, int>::const_iterator Imp = atom_to_index_.begin() ; Imp != atom_to_index_.end() ; ++Imp)
	{
		dd_node_ptr var(Cudd_bddIthVar(dd_node_ptr::get_cudd_manager(), Imp->second), false);
		
		if (st.atoms().find(Imp->first) != st.atoms().end())
		{
			cudd_state = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), cudd_state.get(), var.get()));
			atoms_found.insert(Imp->first);
		}
		else
			cudd_state = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), cudd_state.get(), Cudd_Not(var.get())));
	}
	
	if (atoms_found.size() != st.atoms().size())
	{
		AtomSet new_atoms;
		std::set_difference(st.atoms().begin(), st.atoms().end(),
				atoms_found.begin(), atoms_found.end(),
				std::inserter(new_atoms, new_atoms.begin()));
		
		int index = Cudd_ReadSize(dd_node_ptr::get_cudd_manager());
		new_dd_variables(new_atoms, index);
		
		dd_node_ptr bdd_correction(Cudd_ReadOne(dd_node_ptr::get_cudd_manager()));
		dd_node_ptr add_correction(Cudd_ReadOne(dd_node_ptr::get_cudd_manager()));
		
		dd_node_ptr value_mask_correction(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
		dd_node_ptr transitions_correction(Cudd_ReadOne(dd_node_ptr::get_cudd_manager()));
		
		for (AtomSet::const_iterator Iat = new_atoms.begin() ; Iat != new_atoms.end() ; ++Iat)
		{
			dd_node_ptr bdd_var(Cudd_bddIthVar(dd_node_ptr::get_cudd_manager(), index), false);
			dd_node_ptr add_var(Cudd_addIthVar(dd_node_ptr::get_cudd_manager(), index));
			add_var = dd_node_ptr(Cudd_addCmpl(dd_node_ptr::get_cudd_manager(), add_var.get()));
			
			// update state components
			cudd_state = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), cudd_state.get(), bdd_var.get()));
			
			// update BDD correction
			bdd_correction = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), bdd_correction.get(), Cudd_Not(bdd_var.get())));
			
			// update ADD correction
			add_correction = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, add_correction.get(), add_var.get()));
			
			// update value mask correction
			value_mask_correction = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), value_mask_correction.get(), bdd_var.get()));
			
			// update transitions correction
			dd_node_ptr add_varp(Cudd_addIthVar(dd_node_ptr::get_cudd_manager(), index+1));
			add_varp = dd_node_ptr(Cudd_addCmpl(dd_node_ptr::get_cudd_manager(), add_varp.get()));
			dd_node_ptr and_dd(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, add_var.get(), add_varp.get()));
			transitions_correction = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, transitions_correction.get(), and_dd.get()));
			
			// update index
			index += 2;
		}
		
		// update value mask correction
		value_mask_correction = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), value_mask_correction.get()));
		dd_node_ptr minus_infinity_dd(Cudd_ReadMinusInfinity(dd_node_ptr::get_cudd_manager()));
		value_mask_correction = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, value_mask_correction.get(), minus_infinity_dd.get()));
		
		// Update transitions
		
		for (transitions_map_t::iterator Itr = transitions_.begin() ; Itr != transitions_.end() ; ++Itr)
		{
			Itr->second._policy_mask_() = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), Itr->second.get_policy_mask().get(), bdd_correction.get()));
			Itr->second._value_mask_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, Itr->second.get_value_mask().get(), value_mask_correction.get()));
			Itr->second._probabilities_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, Itr->second.get_probabilities().get(), transitions_correction.get()));
			Itr->second._rewards_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, Itr->second.get_rewards().get(), transitions_correction.get()));
		}
		
		// Update states
		states_ = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), states_.get(), bdd_correction.get()));
		unexpanded_states_ = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), unexpanded_states_.get(), bdd_correction.get()));
		goal_states_ = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), goal_states_.get(), bdd_correction.get()));
		
		if (states_record_)
			states_record_ = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), states_record_.get(), bdd_correction.get()));
		
		if (state_blackbox_)
			state_blackbox_ = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), state_blackbox_.get(), bdd_correction.get()));
		
		// Update connected diagrams
		
		for (std::set<dd_node_ptr*>::iterator Ibdd = connected_bdds_.begin() ; Ibdd != connected_bdds_.end() ; ++Ibdd)
			**Ibdd = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), (*Ibdd)->get(), bdd_correction.get()));
		
		for (std::set<dd_node_ptr*>::iterator Iadd = connected_adds_.begin() ; Iadd != connected_adds_.end() ; ++Iadd)
			**Iadd = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, (*Iadd)->get(), add_correction.get()));
		
		// Update policy, value function, and probability function
		policy_ = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, policy_.get(), add_correction.get()));
		value_function_ = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, value_function_.get(), add_correction.get()));
		probability_function_ = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, probability_function_.get(), add_correction.get()));
	}
	
	dd_node_ptr and_dd(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), states_.get(), cudd_state.get()));
	
	if (and_dd == Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()))
	{
		states_ = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), states_.get(), cudd_state.get()));
		unexpanded_states_ = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), unexpanded_states_.get(), cudd_state.get()));
		
		if (goal)
			goal_states_ = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), goal_states_.get(), cudd_state.get()));
	}
	
	return cudd_state;
}


void SymbolicMDP::new_dd_variables(const AtomSet& new_atoms, int index)
{
	int nb_of_dd_var = index;
	
	for (AtomSet::const_iterator Iat = new_atoms.begin() ; Iat != new_atoms.end() ; ++Iat)
	{
		atom_to_index_.insert(std::make_pair(*Iat, nb_of_dd_var));
		nb_of_dd_var += 2;
	}
	
	if (unprimed_to_primed_permutation_)
		delete[] unprimed_to_primed_permutation_;
	
	if (primed_to_unprimed_permutation_)
		delete[] primed_to_unprimed_permutation_;
	
	unprimed_to_primed_permutation_ = new int[nb_of_dd_var];
	primed_to_unprimed_permutation_ = new int[nb_of_dd_var];
	
	DdNode ** unprimed_variables_bdd_vars = new DdNode*[nb_of_dd_var/2];
	DdNode ** unprimed_variables_add_vars = new DdNode*[nb_of_dd_var/2];
	DdNode ** primed_variables_bdd_vars = new DdNode*[nb_of_dd_var/2];
	DdNode ** primed_variables_add_vars = new DdNode*[nb_of_dd_var/2];
	
	for (int cnt = 0 ; cnt < (nb_of_dd_var/2) ; cnt++)
	{
		unprimed_to_primed_permutation_[2*cnt] = (2*cnt)+1;
		unprimed_to_primed_permutation_[(2*cnt)+1] = (2*cnt)+1;
		primed_to_unprimed_permutation_[2*cnt] = 2*cnt;
		primed_to_unprimed_permutation_[(2*cnt)+1] = 2*cnt;
		
		unprimed_variables_bdd_vars[cnt] = Cudd_bddIthVar(dd_node_ptr::get_cudd_manager(), 2*cnt);
		primed_variables_bdd_vars[cnt] = Cudd_bddIthVar(dd_node_ptr::get_cudd_manager(), (2*cnt)+1);
		unprimed_variables_add_vars[cnt] = Cudd_addIthVar(dd_node_ptr::get_cudd_manager(), 2*cnt);
		primed_variables_add_vars[cnt] = Cudd_addIthVar(dd_node_ptr::get_cudd_manager(), (2*cnt)+1);
		Cudd_Ref(unprimed_variables_add_vars[cnt]);
		Cudd_Ref(primed_variables_add_vars[cnt]);
	}
	
	unprimed_variables_bdd_cube_ = dd_node_ptr(Cudd_bddComputeCube(dd_node_ptr::get_cudd_manager(), unprimed_variables_bdd_vars, NULL, nb_of_dd_var/2));
	unprimed_variables_add_cube_ = dd_node_ptr(Cudd_addComputeCube(dd_node_ptr::get_cudd_manager(), unprimed_variables_add_vars, NULL, nb_of_dd_var/2));
	primed_variables_bdd_cube_ = dd_node_ptr(Cudd_bddComputeCube(dd_node_ptr::get_cudd_manager(), primed_variables_bdd_vars, NULL, nb_of_dd_var/2));
	primed_variables_add_cube_ = dd_node_ptr(Cudd_addComputeCube(dd_node_ptr::get_cudd_manager(), primed_variables_add_vars, NULL, nb_of_dd_var/2));
	
	for (int cnt = 0 ; cnt < (nb_of_dd_var/2) ; cnt++)
	{
		Cudd_RecursiveDeref(dd_node_ptr::get_cudd_manager(), unprimed_variables_add_vars[cnt]);
		Cudd_RecursiveDeref(dd_node_ptr::get_cudd_manager(), primed_variables_add_vars[cnt]);
	}
	
	delete[] unprimed_variables_bdd_vars;
	delete[] unprimed_variables_add_vars;
	delete[] primed_variables_bdd_vars;
	delete[] primed_variables_add_vars;
}


dd_node_ptr SymbolicMDP::add_state(const PddlState& st)
{
	if (tb_.get_problem().goal().holds(tb_.get_problem().terms(), st.atoms(), st.values()))
		return get_and_add_cudd_state(st, true);
	else
		return get_and_add_cudd_state(st, false);
}


void SymbolicMDP::remove_state(const PddlState& st)
{
	dd_node_ptr cudd_state;
	
	if (_cudd_state_(st, cudd_state))
	{
		dd_node_ptr and_dd(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), states_.get(), cudd_state.get()));
		
		if (and_dd.get() == cudd_state.get())
		{
			states_ = dd_node_ptr(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), states_.get(), cudd_state.get()));
			unexpanded_states_ = dd_node_ptr(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), unexpanded_states_.get(), cudd_state.get()));
			dd_node_ptr cudd_state_not(Cudd_Not(cudd_state.get()));
			cudd_state = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), cudd_state.get()));
			dd_node_ptr cudd_state_cmpl = dd_node_ptr(Cudd_addCmpl(dd_node_ptr::get_cudd_manager(), cudd_state.get()));
			dd_node_ptr minus_infinity_dd(Cudd_ReadMinusInfinity(dd_node_ptr::get_cudd_manager()));
			minus_infinity_dd = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, cudd_state.get(), minus_infinity_dd.get()));
			
			for (transitions_map_t::iterator Itr = transitions_.begin() ; Itr != transitions_.end() ; ++Itr)
			{
				Itr->second._policy_mask_() = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), Itr->second.get_policy_mask().get(), cudd_state_not.get()));
				Itr->second._value_mask_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, Itr->second.get_value_mask().get(), minus_infinity_dd.get()));
				Itr->second._probabilities_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, Itr->second.get_probabilities().get(), cudd_state_cmpl.get()));
				Itr->second._rewards_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, Itr->second.get_rewards().get(), cudd_state_cmpl.get()));
				
			}
		}
	}
}


dd_node_ptr SymbolicMDP::get_state(const PddlState& st) const
{
	dd_node_ptr cudd_state;
	bool unknown_state = false;
	
	if (!_cudd_state_(st, cudd_state))
		unknown_state = true;
	
	dd_node_ptr and_dd(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), states_.get(), cudd_state.get()));
	
	if (and_dd == Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()))
		unknown_state = true;
	
	if (unknown_state)
	{
		std::ostringstream state_str;
		state_str << "unknwon state " << st;
		throw BaseException(state_str.str(), "SymbolicMDP::get_state");
	}
	
	return cudd_state;
}


const Action& SymbolicMDP::get_policy(const PddlState& st) const
{
	try
	{
		dd_node_ptr cudd_state = get_cudd_state(st);
		return get_policy(cudd_state);
	}
	catch (BaseException& error)
	{
		error.push_function_backtrace("SymbolicMDP::get_policy");
		throw;
	}
}


const Action& SymbolicMDP::get_policy(const dd_node_ptr& st) const
{
	try
	{
		dd_node_ptr cudd_state(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), st.get()));
		DdNode* action_dd_node = Cudd_addEvalConst(dd_node_ptr::get_cudd_manager(), cudd_state.get(), policy_.get());
		
		if (action_dd_node == DD_NON_CONSTANT)
		{
			std::ostringstream state_str;
			state_str << "state " << st << " is incomplete (it is a set of states)";
			throw BaseException(state_str.str());
		}
		
		dd_node_ptr action_dd(action_dd_node);
		CUDD_VALUE_TYPE action = Cudd_V(action_dd.get());
		std::map<CUDD_VALUE_TYPE, const Action*>::const_iterator Imp = index_to_action_.find(action);
		
		if (Imp == index_to_action_.end())
		{
			std::ostringstream error_str;
			error_str << "policy in state " << st << ": action " << action << " is unknown";
			throw BaseException(error_str.str());
		}
		
		return (*(Imp->second));
	}
	catch (BaseException& error)
	{
		error.push_function_backtrace("SymbolicMDP::get_policy");
		throw;
	}
}


double SymbolicMDP::get_value(const PddlState& st) const
{
	try
	{
		dd_node_ptr cudd_state = get_cudd_state(st);
		return get_value(st);
	}
	catch (BaseException& error)
	{
		error.push_function_backtrace("SymbolicMDP::get_value");
		throw;
	}
}


double SymbolicMDP::get_value(const dd_node_ptr& st) const
{
	try
	{
		dd_node_ptr cudd_state(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), st.get()));
		DdNode* value_dd_node = Cudd_addEvalConst(dd_node_ptr::get_cudd_manager(), cudd_state.get(), value_function_.get());
		
		if (value_dd_node == DD_NON_CONSTANT)
		{
			std::ostringstream state_str;
			state_str << "state " << st << " is incomplete (it is a set of states)";
			throw BaseException(state_str.str());
		}
		
		dd_node_ptr value_dd(value_dd_node);
		return Cudd_V(value_dd.get());
	}
	catch (BaseException& error)
	{
		error.push_function_backtrace("SymbolicMDP::get_value");
		throw;
	}
}


double SymbolicMDP::get_probability(const PddlState& st) const
{
	try
	{
		dd_node_ptr cudd_state = get_cudd_state(st);
		return get_probability(st);
	}
	catch (BaseException& error)
	{
		error.push_function_backtrace("SymbolicMDP::get_probability");
		throw;
	}
}


double SymbolicMDP::get_probability(const dd_node_ptr& st) const
{
	try
	{
		dd_node_ptr cudd_state(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), st.get()));
		DdNode* probability_dd_node = Cudd_addEvalConst(dd_node_ptr::get_cudd_manager(), cudd_state.get(), probability_function_.get());
		
		if (probability_dd_node == DD_NON_CONSTANT)
		{
			std::ostringstream state_str;
			state_str << "state " << st << " is incomplete (it is a set of states)";
			throw BaseException(state_str.str());
		}
		
		dd_node_ptr probability_dd(probability_dd_node);
		return Cudd_V(probability_dd.get());
	}
	catch (BaseException& error)
	{
		error.push_function_backtrace("SymbolicMDP::get_probability");
		throw;
	}
}


bool SymbolicMDP::explored(const PddlState& st) const
{
	dd_node_ptr cudd_state;
	
	if (!_cudd_state_(st, cudd_state))
		return false;
	
	dd_node_ptr and_dd(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), states_.get(), cudd_state.get()));
	return (and_dd.get() != Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
}


dd_node_ptr SymbolicMDP::explored(const dd_node_ptr& states) const
{
	return dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), states_.get(), states.get()));
}


bool SymbolicMDP::expanded(const PddlState& st) const
{
	dd_node_ptr cudd_state;
	
	if (!_cudd_state_(st, cudd_state))
		return false;
	
	dd_node_ptr and_dd(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), states_.get(), cudd_state.get()));
	
	if (and_dd.get() == Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()))
		return false;
	
	and_dd = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), unexpanded_states_.get(), cudd_state.get()));
	return (and_dd == Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
}


dd_node_ptr SymbolicMDP::expanded(const dd_node_ptr& states) const
{
	dd_node_ptr expanded_states(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), states_.get(), unexpanded_states_.get()));
	return dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), expanded_states.get(), states.get()));
}


bool SymbolicMDP::goal(const PddlState& st) const
{
	dd_node_ptr cudd_st;
	
	if (!_cudd_state_(st, cudd_st))
	{
		const Problem& problem = tb_.get_problem();
		const StateFormula& goal_formula = problem.goal();
	
		return goal_formula.holds(problem.terms(), st.atoms(), st.values());
	}
	else
	{
		dd_node_ptr and_dd(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), goal_states_.get(), cudd_st.get()));
		return (and_dd.get() != Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	}
}


dd_node_ptr SymbolicMDP::goals(const dd_node_ptr& states) const
{
	return dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), goal_states_.get(), states.get()));
}


dd_node_ptr SymbolicMDP::policy_defined(const dd_node_ptr& states) const
{
	dd_node_ptr policy_bdd(Cudd_addBddPattern(dd_node_ptr::get_cudd_manager(), policy_.get()));
	return dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), policy_bdd.get(), states.get()));
}


const StateFormula& SymbolicMDP::state_formula(const dd_node_ptr& states) const
{
	std::list<PddlState> pddl_states;
	decompose(states, pddl_states);
	const StateFormula* res = &state_formula_false;
	
	for (std::list<PddlState>::const_iterator Ips = pddl_states.begin() ; Ips != pddl_states.end() ; ++Ips)
		res = &((*res) || (Ips->state_formula()));
	
	return (*res);
}


std::string SymbolicMDP::print_model() const
{
	std::ostringstream model_str;
	model_str << "----------------------------------------" << std::endl;
	model_str << "MDP model:\n" << std::endl;
	
	int nb_of_dd_var = Cudd_ReadSize(dd_node_ptr::get_cudd_manager());
	model_str << "Number of DD variables: " << nb_of_dd_var << " (" << (nb_of_dd_var/2) << " PDDL predicates instantiated)" << std::endl;
	model_str << "Predicates to DD variables mapping: " << std::endl;
	
	for (std::map<const Atom*, int>::const_iterator Imp = atom_to_index_.begin() ; Imp != atom_to_index_.end() ; ++Imp)
		model_str << "atom " << (*(Imp->first)) << ": " << (Imp->second) << std::endl;
	
	model_str << "\nActions:\n" << std::endl;
	
	for (transitions_map_t::const_iterator Itr = transitions_.begin() ; Itr != transitions_.end() ; ++Itr)
	{
		model_str << "Action " << (*(Itr->first)) << ":" << std::endl;
		model_str << "\nProbabilities:" << std::endl;
		cudd_print_to_ostream(Itr->second.get_probabilities(), model_str);
		model_str << "\nRewards:" << std::endl;
		cudd_print_to_ostream(Itr->second.get_rewards(), model_str);
		model_str << "\nPolicy mask:" << std::endl;
		cudd_print_to_ostream(Itr->second.get_policy_mask(), model_str);
		model_str << "\nValue mask:" << std::endl;
		cudd_print_to_ostream(Itr->second.get_value_mask(), model_str);
		model_str << std::endl;
	}
	
	model_str << "----------------------------------------" << std::endl;
	return model_str.str();
}


std::string SymbolicMDP::print_solution() const
{
	std::ostringstream solution_str;
	solution_str << "----------------------------------------" << std::endl;
	solution_str << "MDP solution:\n" << std::endl;
	
	if (!value_function_)
		solution_str << "MDP solution not yet computed" << std::endl;
	else
	{
		solution_str << "Policy:" << std::endl;
		cudd_print_to_ostream(policy_, solution_str);
		
		solution_str << "\nValue function:" << std::endl;
		cudd_print_to_ostream(value_function_, solution_str);

		solution_str << "\nProbability function:" << std::endl;
		cudd_print_to_ostream(probability_function_, solution_str);
	}
	
	solution_str << "----------------------------------------" << std::endl;
	return solution_str.str();
}


void SymbolicMDP::cudd_print_to_ostream(const dd_node_ptr& dd, std::ostream& output_str) const
{
	int fd[2];
	pipe(fd);
	FILE* myout = fdopen(fd[1], "w");
	FILE* backed_stdout = Cudd_ReadStdout(dd_node_ptr::get_cudd_manager());
	Cudd_SetStdout(dd_node_ptr::get_cudd_manager(), myout);
	Cudd_PrintMinterm(dd_node_ptr::get_cudd_manager(), dd.get());
	fclose(myout);
	FILE* myin = fdopen(fd[0], "r");
	char c;
	
	while (true)
	{
		c = fgetc(myin);
		
		if (c != EOF)
			output_str.put(c);
		else
			break;
	}
	
	fclose(myin);
	Cudd_SetStdout(dd_node_ptr::get_cudd_manager(), backed_stdout);
	close (fd[0]);
	close (fd[1]);
}


// CLASS ScopedConnectionDiagram
SymbolicMDP* ScopedConnectionDiagram::mdp_ = NULL;
