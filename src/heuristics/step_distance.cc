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

#include <algorithm>
#include <limits>

#include "step_distance.h"

// CLASS PddlStepDistance

PddlStepDistance::PddlStepDistance(double discount_factor, const TransitionsBlackbox& tb)
try : BaseHeuristic(discount_factor), TransitionsBlackboxUser(tb)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("StepDistance::StepDistance");
	throw;
}


void PddlStepDistance::clear_states()
{
	for (states_set_t::iterator Ist = states_.begin() ; Ist != states_.end() ; ++Ist)
		delete (*Ist);

	states_.clear();
}


void PddlStepDistance::transition_blackbox(const Action& action, double probability, double reward, PddlState* outcome, bool goal)
{
	std::pair<states_set_t::iterator, bool> Ist = states_.insert(outcome);

	if (Ist.second)
	{
		new_states_.insert(outcome);

		if (!goal_state_reached_)
			goal_state_reached_ = goal;
	}
	else
		delete outcome;
}


double PddlStepDistance::value(const PddlState& st)
{
	clear_states();
	const PddlState* nst = *(states_.insert(st.copy()).first);
	states_set_t states_frontier;
	states_frontier.insert(nst);
	goal_state_reached_ = false;
	double val = 0.0;

	while (!states_frontier.empty())
	{
		new_states_.clear();

		for (states_set_t::const_iterator Isf = states_frontier.begin() ; Isf != states_frontier.end() ; ++Isf)
			tb_.transitions(**Isf, *this);

		states_frontier = new_states_;

		if (goal_state_reached_)
			break;
		else
			val = (discount_factor_ * val) - 1.0;
	}

	new_states_.clear();
	clear_states();

	if (goal_state_reached_)
		return val;
	else
		return ((discount_factor_ < 1.0)?(-1.0 / (1.0 - discount_factor_)):(-std::numeric_limits<double>::max()));
}




// CLASS PddlGraphStepDistance

template <class Tstate>
PddlGraphStepDistance<Tstate>::PddlGraphStepDistance(GraphMDP<Tstate>& mdp, double discount_factor, const TransitionsBlackbox& tb)
try : BaseHeuristic(discount_factor), GraphHeuristic<Tstate>(mdp, discount_factor), PddlStepDistance(discount_factor, tb)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("PddlGraphStepDistance::PddlGraphStepDistance");
	throw;
}


template <class Tstate>
void PddlGraphStepDistance<Tstate>::compute(Tstate& st)
{
	st.set_value(value(st.get_pddl_state()));
}


// Explicit template instantiations
INSTANTIATE_STATE_TEMPLATE(PddlGraphStepDistance)




// CLASS NativeGraphStepDistance

template <class Tstate>
NativeGraphStepDistance<Tstate>::NativeGraphStepDistance(GraphMDP<Tstate>& mdp, double discount_factor)
try : BaseHeuristic(discount_factor), GraphHeuristic<Tstate>(mdp, discount_factor)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("NativeGraphStepDistance::NativeGraphStepDistance");
	throw;
}


template <class Tstate>
void NativeGraphStepDistance<Tstate>::compute(Tstate& st)
{
	if ((initialized_states_.find(&st)) != (initialized_states_.end()))
		return;

	states_set_t forward_states;
	states_set_t states_frontier;
	states_set_t goal_states;
	forward_states.insert(&st);
	states_frontier.insert(&st);

	// forward search

	while ((!states_frontier.empty()) && (goal_states.empty()))
	{
		states_set_t next_states;

		for (typename states_set_t::iterator Isf = states_frontier.begin() ; Isf != states_frontier.end() ; ++Isf)
		{
			const typename GraphMDP<Tstate>::actions_map_t* transitions = this->mdp_.transitions(**Isf);

			for (typename GraphMDP<Tstate>::actions_map_t::const_iterator Iac = transitions->begin() ; Iac != transitions->end() ; ++Iac)
			{
				for (typename GraphMDP<Tstate>::transition_list_t::const_iterator Itr = Iac->second.begin() ; Itr != Iac->second.end() ; ++Itr)
				{
					Tstate& next_state = (*Itr)->_outcome_();
					next_states.insert(&next_state);

					if (this->mdp_.goal(next_state))
						goal_states.insert(&next_state);

					_add_ancestor_(**Isf, next_state, *(Iac->first), **Itr);
				}
			}

			_delete_transitions_(transitions);
		}

		states_frontier.clear();
		std::set_difference(next_states.begin(), next_states.end(),
				forward_states.begin(), forward_states.end(),
				std::inserter(states_frontier, states_frontier.begin()));
		states_set_t temp_set;
		std::set_union(forward_states.begin(), forward_states.end(),
				next_states.begin(), next_states.end(),
				std::inserter(temp_set, temp_set.begin()));
		forward_states.swap(temp_set);
	}

	// backward update

	states_frontier = goal_states;
	states_set_t backward_states = goal_states;
	bool st_reached = false;
	double val = 0.0;

	while ((!states_frontier.empty()) && (!st_reached))
	{
		states_set_t previous_states;
		val = ((this->discount_factor_) * val) - 1.0;

		for (typename states_set_t::iterator Isf = states_frontier.begin() ; Isf != states_frontier.end() ; ++Isf)
		{
			ancestors_map_t& ancestors = _ancestors_(**Isf);

			for (typename ancestors_map_t::iterator Ian = ancestors.begin() ; Ian != ancestors.end() ; ++Ian)
			{
				Tstate& ancestor_st = *(Ian->first);

				if (forward_states.find(&ancestor_st) != forward_states.end())
				{
					previous_states.insert(&ancestor_st);

					if (&ancestor_st == &st)
						st_reached = true;

					if ((initialized_states_.find(&ancestor_st)) == (initialized_states_.end()))
					{
						ancestor_st.set_value(val);
						this->initialized_states_.insert(&ancestor_st);
					}
				}
			}
		}

		states_frontier.clear();
		std::set_difference(previous_states.begin(), previous_states.end(),
				backward_states.begin(), backward_states.end(),
				std::inserter(states_frontier, states_frontier.begin()));
		states_set_t temp_set;
		std::set_union(backward_states.begin(), backward_states.end(),
				previous_states.begin(), previous_states.end(),
				std::inserter(temp_set, temp_set.begin()));
		backward_states.swap(temp_set);
	}

	initialized_states_.insert(&st);

	if (!st_reached)
		st.set_value(-1.0 / (1.0 - (this->discount_factor_)));
}


#define INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_GS(StateT) \
template <> \
void NativeGraphStepDistance<StateT>::_delete_transitions_(const GraphMDP<StateT>::actions_map_t* transitions) \
{ \
	delete transitions; \
} \
template <> \
void NativeGraphStepDistance<StateT>::_add_ancestor_(StateT& income, StateT& outcome, const Action& action, const GraphTransition<StateT>& transition) \
{ \
	states_ancestors_[&outcome][&income][&action] = &transition; \
} \
template <> \
NativeGraphStepDistance<StateT>::ancestors_map_t& NativeGraphStepDistance<StateT>::_ancestors_(StateT& st) \
{ \
	return states_ancestors_[&st]; \
}

INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_GS(GraphState)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_GS(GraphStateProbability)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_GS(GraphStateVariance)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_GS(GraphStateSolveBit)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_GS(GraphStateConnection)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_GS(GraphStateSolveBitConnection)


#define INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WT(StateT) \
template <> \
void NativeGraphStepDistance<GraphStateWT<StateT> >::_delete_transitions_(const GraphMDP<GraphStateWT<StateT> >::actions_map_t* transitions) \
{ \
} \
template <> \
void NativeGraphStepDistance<GraphStateWT<StateT> >::_add_ancestor_(GraphStateWT<StateT>& income, GraphStateWT<StateT>& outcome, const Action& action, const GraphTransition<GraphStateWT<StateT> >& transition) \
{ \
	states_ancestors_[&outcome][&income][&action] = &transition; \
} \
template <> \
NativeGraphStepDistance<GraphStateWT<StateT> >::ancestors_map_t& NativeGraphStepDistance<GraphStateWT<StateT> >::_ancestors_(GraphStateWT<StateT>& st) \
{ \
	return states_ancestors_[&st]; \
}

INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WT(GraphState)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WT(GraphStateProbability)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WT(GraphStateVariance)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WT(GraphStateSolveBit)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WT(GraphStateConnection)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WT(GraphStateSolveBitConnection)


#define INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WA(StateT) \
template <> \
void NativeGraphStepDistance<GraphStateWA<StateT> >::_delete_transitions_(const GraphMDP<GraphStateWA<StateT> >::actions_map_t* transitions) \
{ \
	delete transitions; \
} \
template <> \
void NativeGraphStepDistance<GraphStateWA<StateT> >::_add_ancestor_(GraphStateWA<StateT>& income, GraphStateWA<StateT>& outcome, const Action& action, const GraphTransition<GraphStateWA<StateT> >& transition) \
{ \
} \
template <> \
NativeGraphStepDistance<GraphStateWA<StateT> >::ancestors_map_t& NativeGraphStepDistance<GraphStateWA<StateT> >::_ancestors_(GraphStateWA<StateT>& st) \
{ \
	return st._ancestors_(); \
}

INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WA(GraphState)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WA(GraphStateProbability)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WA(GraphStateVariance)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WA(GraphStateSolveBit)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WA(GraphStateConnection)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WA(GraphStateSolveBitConnection)


#define INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WTA(StateT) \
template <> \
void NativeGraphStepDistance<GraphStateWTA<StateT> >::_delete_transitions_(const GraphMDP<GraphStateWTA<StateT> >::actions_map_t* transitions) \
{ \
} \
template <> \
void NativeGraphStepDistance<GraphStateWTA<StateT> >::_add_ancestor_(GraphStateWTA<StateT>& income, GraphStateWTA<StateT>& outcome, const Action& action, const GraphTransition<GraphStateWTA<StateT> >& transition) \
{ \
} \
template <> \
NativeGraphStepDistance<GraphStateWTA<StateT> >::ancestors_map_t& NativeGraphStepDistance<GraphStateWTA<StateT> >::_ancestors_(GraphStateWTA<StateT>& st) \
{ \
	return st._ancestors_(); \
}

INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WTA(GraphState)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WTA(GraphStateProbability)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WTA(GraphStateVariance)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WTA(GraphStateSolveBit)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WTA(GraphStateConnection)
INSTANTIATE_NATIVE_GRAPH_STEP_DISTANCE_METHODS_WTA(GraphStateSolveBitConnection)


// Explicit template instantiations
INSTANTIATE_STATE_TEMPLATE(NativeGraphStepDistance)


// CLASS PddlSymbolicStepDistance

PddlSymbolicStepDistance::PddlSymbolicStepDistance(SymbolicMDP& mdp, double discount_factor, const TransitionsBlackbox& tb)
try : BaseHeuristic(discount_factor), SymbolicHeuristic(mdp, discount_factor), PddlStepDistance(discount_factor, tb)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("PddlSymbolicStepDistance::PddlSymbolicStepDistance");
	throw;
}


void PddlSymbolicStepDistance::compute(const dd_node_ptr& states)
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




// CLASS NativeSymbolicStepDistance

NativeSymbolicStepDistance::NativeSymbolicStepDistance(SymbolicMDP& mdp, double discount_factor)
try : BaseHeuristic(discount_factor), SymbolicHeuristic(mdp, discount_factor)
{
	initialized_states_ = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	mdp.connect_bdd(initialized_states_);
}
catch (BaseException& error)
{
	error.push_function_backtrace("NativeSymbolicStepDistance::NativeSymbolicStepDistance");
	throw;
}


NativeSymbolicStepDistance::~NativeSymbolicStepDistance()
{
	mdp_.disconnect_bdd(initialized_states_);
}


void NativeSymbolicStepDistance::compute(const dd_node_ptr& states)
{
	dd_node_ptr states_to_initialize(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), initialized_states_.get(), states.get()));
	states_to_initialize = dd_node_ptr(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), states_to_initialize.get(), states.get()));

	std::list<PddlState> pddl_states;
	mdp_.decompose(states_to_initialize, pddl_states);

	for (std::list<PddlState>::const_iterator Ist = pddl_states.begin() ; Ist != pddl_states.end() ; ++Ist)
		compute_single_state(mdp_.get_state(*Ist));
}


void NativeSymbolicStepDistance::compute_single_state(const dd_node_ptr& st)
{
	dd_node_ptr forward_states;
	ScopedConnectionBDD forward_states_connection(forward_states);
	forward_states.copy(st);
	dd_node_ptr states_frontier;
	ScopedConnectionBDD states_frontier_connection(states_frontier);
	states_frontier.copy(st);
	dd_node_ptr reachable_goal_states(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	ScopedConnectionBDD reachable_goal_states_connection(reachable_goal_states);

	while ((states_frontier.get() != Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()))
			&& (reachable_goal_states == Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager())))
	{
		dd_node_ptr new_states_frontier(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
		ScopedConnectionBDD new_states_frontier_connection(new_states_frontier);

		for (SymbolicMDP::transitions_map_t::const_iterator Itr = mdp_.get_transitions().begin() ; Itr != mdp_.get_transitions().end() ; ++Itr)
		{
			dd_node_ptr deterministic_transitions(Cudd_addBddPattern(dd_node_ptr::get_cudd_manager(), Itr->second.get_probabilities().get()));
			dd_node_ptr next_states(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), deterministic_transitions.get(), states_frontier.get()));
			next_states = dd_node_ptr(Cudd_bddExistAbstract(dd_node_ptr::get_cudd_manager(), next_states.get(), mdp_.get_unprimed_variables_bdd_cube().get()));
			next_states = dd_node_ptr(Cudd_bddPermute(dd_node_ptr::get_cudd_manager(), next_states.get(), mdp_.get_primed_to_unprimed_permutation()));
			dd_node_ptr states_to_expand(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), mdp_.get_unexpanded_states().get(), next_states.get()));
			ScopedConnectionBDD next_states_connection(next_states);

			if (states_to_expand.get() != Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()))
				mdp_.expand(states_to_expand);

			dd_node_ptr and_dd(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), next_states.get(), forward_states.get()));
			next_states = dd_node_ptr(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), next_states.get(), and_dd.get()));
			new_states_frontier = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), new_states_frontier.get(), next_states.get()));
		}

		states_frontier = new_states_frontier;
		forward_states = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), forward_states.get(), states_frontier.get()));
		reachable_goal_states = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), states_frontier.get(), mdp_.get_goal_states().get()));
	}

	dd_node_ptr backward_states;
	backward_states.copy(reachable_goal_states);
	states_frontier.copy(reachable_goal_states);
	dd_node_ptr new_initialized_states(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	dd_node_ptr heuristic_values(Cudd_ReadZero(dd_node_ptr::get_cudd_manager()));
	bool st_reached = false;
	double val = 0.0;

	while ((backward_states.get() != Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager())) && (!st_reached))
	{
		val = (discount_factor_ * val) - 1.0;
		dd_node_ptr val_dd(Cudd_addConst(dd_node_ptr::get_cudd_manager(), val));
		dd_node_ptr new_states_frontier(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
		states_frontier = dd_node_ptr(Cudd_bddPermute(dd_node_ptr::get_cudd_manager(), states_frontier.get(), mdp_.get_unprimed_to_primed_permutation()));

		for (SymbolicMDP::transitions_map_t::const_iterator Itr = mdp_.get_transitions().begin() ; Itr != mdp_.get_transitions().end() ; ++Itr)
		{
			dd_node_ptr deterministic_transitions(Cudd_addBddPattern(dd_node_ptr::get_cudd_manager(), Itr->second.get_probabilities().get()));
			dd_node_ptr previous_states(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), deterministic_transitions.get(), states_frontier.get()));
			previous_states = dd_node_ptr(Cudd_bddExistAbstract(dd_node_ptr::get_cudd_manager(), previous_states.get(), mdp_.get_primed_variables_bdd_cube().get()));
			previous_states = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), previous_states.get(), forward_states.get()));
			dd_node_ptr and_dd(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), previous_states.get(), backward_states.get()));
			previous_states = dd_node_ptr(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), previous_states.get(), and_dd.get()));
			new_states_frontier = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), new_states_frontier.get(), previous_states.get()));
		}

		states_frontier = new_states_frontier;
		dd_node_ptr and_dd(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), states_frontier.get(), initialized_states_.get()));
		dd_node_ptr states_to_initialize(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), states_frontier.get(), and_dd.get()));
		new_initialized_states = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), new_initialized_states.get(), states_to_initialize.get()));
		initialized_states_ = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), initialized_states_.get(), states_to_initialize.get()));
		states_to_initialize = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), states_to_initialize.get()));
		states_to_initialize = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, states_to_initialize.get(), val_dd.get()));
		heuristic_values = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, heuristic_values.get(), states_to_initialize.get()));

		and_dd = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), states_frontier.get(), st.get()));
		st_reached = (and_dd != Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	}

	initialized_states_ = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), initialized_states_.get(), st.get()));
	new_initialized_states = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), new_initialized_states.get(), st.get()));

	if (!st_reached)
	{
		dd_node_ptr valdd(Cudd_addConst(dd_node_ptr::get_cudd_manager(), (discount_factor_ < 1.0)?(-1.0 / (1.0 - discount_factor_)):(-std::numeric_limits<double>::max())));
		dd_node_ptr st_add(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), st.get()));
		st_add = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, st_add.get(), valdd.get()));
		heuristic_values = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, heuristic_values.get(), st_add.get()));
	}

	dd_node_ptr initialized_states_cmpl(Cudd_Not(new_initialized_states.get()));
	initialized_states_cmpl = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), initialized_states_cmpl.get()));
	mdp_._value_function_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, mdp_.get_value_function().get(), initialized_states_cmpl.get()));
	mdp_._value_function_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, mdp_.get_value_function().get(), heuristic_values.get()));
}
