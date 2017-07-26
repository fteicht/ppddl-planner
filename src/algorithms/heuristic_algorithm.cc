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

#include "heuristic_algorithm.h"
#include "heuristics/zero_heuristic.h"
#include "heuristics/step_distance.h"
#include "heuristics/relaxed_step_distance.h"
#include "heuristics/relaxed_rewards.h"
#include "heuristics/relaxed_planning_graph.h"
#include "heuristics/graph_bounds.h"
#include "heuristics/symbolic_bounds.h"
#include "heuristics/hadd.h"
#include "heuristics/hmax.h"
#include "heuristics/hdet.h"
#include "commons/base_exceptions.h"

// CLASS HeuristicAlgorithm

HeuristicAlgorithm::HeuristicAlgorithm(const Problem& pb, double epsilon, double discount_factor, heuristic_t heuristic_type)
try : BaseAlgorithm(pb, epsilon, discount_factor), heuristic_type_(heuristic_type)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("HeuristicAlgorithm::HeuristicAlgorithm");
	throw;
}


bool HeuristicAlgorithm::policy_defined(const PddlState& st)
{
	return ((get_mdp().explored(st)) && (get_mdp().expanded(st)));
}




// CLASS GraphHeuristicAlgorithm

template <class Tstate>
GraphHeuristicAlgorithm<Tstate>::GraphHeuristicAlgorithm(const Problem& pb, double epsilon, double discount_factor, unsigned int plan_length, heuristic_t heuristic_type,
															determinization_t determinization_type, deterministic_planner_t deterministic_planner_type)
try : BaseAlgorithm(pb, epsilon, discount_factor), HeuristicAlgorithm(pb, epsilon, discount_factor, heuristic_type), GraphAlgorithm<Tstate>(pb, epsilon, discount_factor), initial_state_(NULL)
{
	switch (heuristic_type)
	{
	case HEURISTIC_ZERO :
		heuristic_ = new GraphZeroHeuristic<Tstate>(*(this->mdp_), discount_factor);
		this->tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);//GOAL);
		break;

	case HEURISTIC_PDDL_STEP_DISTANCE :
		heuristic_ = new PddlGraphStepDistance<Tstate>(*(this->mdp_), discount_factor, *tb_);
		this->tb_->set_metric(TransitionsBlackbox::METRIC_GOAL);
		break;

	case HEURISTIC_NATIVE_STEP_DISTANCE :
		heuristic_ = new NativeGraphStepDistance<Tstate>(*(this->mdp_), discount_factor);
		this->tb_->set_metric(TransitionsBlackbox::METRIC_GOAL);
		break;

	case HEURISTIC_RELAXED_STEP_DISTANCE :
		heuristic_ = new GraphRelaxedStepDistance<Tstate>(*(this->mdp_), discount_factor, tb_->get_problem());
		this->tb_->set_metric(TransitionsBlackbox::METRIC_GOAL);
		break;

	case HEURISTIC_RELAXED_REWARDS :
		heuristic_ = new GraphRelaxedRewards<Tstate>(*(this->mdp_), discount_factor, tb_->get_problem());
		this->tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);
		break;

	case HEURISTIC_RELAXED_PLANNING_GRAPH :
		heuristic_ = new GraphRelaxedPlanningGraph<Tstate>(*(this->mdp_), discount_factor, tb_->get_problem());
		this->tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);
		break;

	case HEURISTIC_BOUNDS:
		heuristic_ = new graph_bounds<Tstate>(*(this->mdp_), discount_factor, pb, pb.actions());
		this->tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);
		break;

	case HEURISTIC_HMAX :
		heuristic_ = new GraphHeuristicHmax<Tstate>(*(this->mdp_), discount_factor, tb_->get_problem());
		this->tb_->set_metric(TransitionsBlackbox::METRIC_GOAL);
		break;

	case HEURISTIC_HADD :
		heuristic_ = new GraphHeuristicHadd<Tstate>(*(this->mdp_), discount_factor, tb_->get_problem());
		this->tb_->set_metric(TransitionsBlackbox::METRIC_GOAL);
		break;

	case HEURISTIC_HDET :
		heuristic_ = new GraphHeuristicHdet<Tstate>(*(this->mdp_), pb, discount_factor, plan_length, determinization_type, deterministic_planner_type);
		this->tb_->set_metric(TransitionsBlackbox::METRIC_GOAL);
		break;

	default :
		throw IncompatibilityException("unexpected heuristic");
	}
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphHeuristicAlgorithm::GraphHeuristicAlgorithm");
	throw;
}


template <class Tstate>
GraphHeuristicAlgorithm<Tstate>::~GraphHeuristicAlgorithm()
{
	delete heuristic_;
}


template <class Tstate>
void GraphHeuristicAlgorithm<Tstate>::solve_initialize(const PddlState& st)
{
	initial_state_ = &(this->mdp_->add_state(st));
	initialize(*initial_state_);
}


template <class Tstate>
void GraphHeuristicAlgorithm<Tstate>::initialize(Tstate& st)
{
	const typename GraphMDP<Tstate>::actions_map_t* transitions = this->mdp_->transitions(st);

	for (typename GraphMDP<Tstate>::new_states_list_t::const_iterator Inst = this->mdp_->new_states().begin() ; Inst != this->mdp_->new_states().end() ; ++Inst)
		heuristic_->compute(*((*Inst)->second));

	_delete_transitions_(transitions);
}


#define INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_GS(StateT) \
template <> \
void GraphHeuristicAlgorithm<StateT>::_delete_transitions_(const GraphMDP<StateT>::actions_map_t* transitions) \
{ \
	delete transitions; \
}

INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_GS(GraphState)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_GS(GraphStateProbability)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_GS(GraphStateVariance)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_GS(GraphStateSolveBit)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_GS(GraphStateConnection)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_GS(GraphStateSolveBitConnection)


#define INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WT(StateT) \
template <> \
void GraphHeuristicAlgorithm<GraphStateWT<StateT> >::_delete_transitions_(const GraphMDP<GraphStateWT<StateT> >::actions_map_t* transitions) \
{ \
}

INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WT(GraphState)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WT(GraphStateProbability)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WT(GraphStateVariance)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WT(GraphStateSolveBit)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WT(GraphStateConnection)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WT(GraphStateSolveBitConnection)


#define INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WA(StateT) \
template <> \
void GraphHeuristicAlgorithm<GraphStateWA<StateT> >::_delete_transitions_(const GraphMDP<GraphStateWA<StateT> >::actions_map_t* transitions) \
{ \
	delete transitions; \
}

INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WA(GraphState)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WA(GraphStateProbability)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WA(GraphStateVariance)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WA(GraphStateSolveBit)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WA(GraphStateConnection)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WA(GraphStateSolveBitConnection)


#define INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WTA(StateT) \
template <> \
void GraphHeuristicAlgorithm<GraphStateWTA<StateT> >::_delete_transitions_(const GraphMDP<GraphStateWTA<StateT> >::actions_map_t* transitions) \
{ \
}

INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WTA(GraphState)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WTA(GraphStateProbability)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WTA(GraphStateVariance)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WTA(GraphStateSolveBit)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WTA(GraphStateConnection)
INSTANTIATE_GRAPH_HEURISTIC_ALGORITHM_DELETE_TRANSITIONS_WTA(GraphStateSolveBitConnection)


// Explicit template instantiations
INSTANTIATE_STATE_TEMPLATE(GraphHeuristicAlgorithm)




// CLASS SymbolicHeuristicAlgorithm

SymbolicHeuristicAlgorithm::SymbolicHeuristicAlgorithm(const Problem& pb, double epsilon, double discount_factor, unsigned int plan_length, heuristic_t heuristic_type,
														determinization_t determinization_type, deterministic_planner_t deterministic_planner_type)
try : BaseAlgorithm(pb, epsilon, discount_factor), HeuristicAlgorithm(pb, epsilon, discount_factor, heuristic_type), SymbolicAlgorithm(pb, epsilon, discount_factor)
{
	switch (heuristic_type)
	{
	case HEURISTIC_ZERO :
		heuristic_ = new SymbolicZeroHeuristic(*mdp_, discount_factor);
		this->tb_->set_metric(TransitionsBlackbox::METRIC_GOAL);
		break;

	case HEURISTIC_PDDL_STEP_DISTANCE :
		heuristic_ = new PddlSymbolicStepDistance(*mdp_, discount_factor, *tb_);
		this->tb_->set_metric(TransitionsBlackbox::METRIC_GOAL);
		break;

	case HEURISTIC_NATIVE_STEP_DISTANCE :
		heuristic_ = new NativeSymbolicStepDistance(*mdp_, discount_factor);
		this->tb_->set_metric(TransitionsBlackbox::METRIC_GOAL);
		break;

	case HEURISTIC_RELAXED_STEP_DISTANCE :
		heuristic_ = new SymbolicRelaxedStepDistance(*mdp_, discount_factor, tb_->get_problem());
		this->tb_->set_metric(TransitionsBlackbox::METRIC_GOAL);
		break;

	case HEURISTIC_RELAXED_REWARDS :
		heuristic_ = new SymbolicRelaxedRewards(*mdp_, discount_factor, tb_->get_problem());
		this->tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);
		break;

	case HEURISTIC_RELAXED_PLANNING_GRAPH :
		heuristic_ = new SymbolicRelaxedPlanningGraph(*mdp_, discount_factor, tb_->get_problem());
		this->tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);
		break;

	case HEURISTIC_BOUNDS:
		heuristic_ = new symbolic_bounds(*mdp_, discount_factor);
		this->tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);
		break;

	case HEURISTIC_HMAX :
		heuristic_ = new SymbolicHeuristicHmax(*mdp_, discount_factor, tb_->get_problem());
		this->tb_->set_metric(TransitionsBlackbox::METRIC_GOAL);
		break;

	case HEURISTIC_HADD :
		heuristic_ = new SymbolicHeuristicHadd(*mdp_, discount_factor, tb_->get_problem());
		this->tb_->set_metric(TransitionsBlackbox::METRIC_GOAL);
		break;

	case HEURISTIC_HDET :
		heuristic_ = new SymbolicHeuristicHdet(*mdp_, pb, discount_factor, plan_length, determinization_type, deterministic_planner_type);
		this->tb_->set_metric(TransitionsBlackbox::METRIC_GOAL);
		break;

	default :
		throw IncompatibilityException("unexpected heuristic");
	}

	initial_state_ = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	explored_states_ = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	tip_states_ = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	mdp_->connect_bdd(initial_state_);
	mdp_->connect_bdd(explored_states_);
	mdp_->connect_bdd(tip_states_);
}
catch (BaseException& error)
{
	error.push_function_backtrace("SymbolicHeuristicAlgorithm::SymbolicHeuristicAlgorithm");
	throw;
}


SymbolicHeuristicAlgorithm::~SymbolicHeuristicAlgorithm()
{
	mdp_->disconnect_bdd(initial_state_);
	mdp_->disconnect_bdd(explored_states_);
	mdp_->disconnect_bdd(tip_states_);
	delete heuristic_;
}


void SymbolicHeuristicAlgorithm::solve_initialize(const PddlState& st)
{
	//explored_states_ = dd_node_ptr(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
	initial_state_ = mdp_->add_state(st);
	//explored_states_.copy(initial_state_);
	//tip_states_.copy(initial_state_);
	explored_states_ = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), explored_states_.get(), initial_state_.get()));
	tip_states_ = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), tip_states_.get(), initial_state_.get()));
	initialize(initial_state_);
	mdp_->connect_bdd(initial_state_); // initialize disconnect initial_state_

	if (!(mdp_->get_value_function()))
		mdp_->_value_function_() = dd_node_ptr(Cudd_ReadZero(dd_node_ptr::get_cudd_manager()));

	if (!(mdp_->get_policy()))
		mdp_->_policy_() = dd_node_ptr(Cudd_ReadZero(dd_node_ptr::get_cudd_manager()));

	if (!discount_factor_dd_)
		discount_factor_dd_ = dd_node_ptr(Cudd_addConst(dd_node_ptr::get_cudd_manager(), discount_factor_));
}


dd_node_ptr SymbolicHeuristicAlgorithm::explored(const dd_node_ptr& states)
{
	return dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), explored_states_.get(), states.get()));
}


bool SymbolicHeuristicAlgorithm::explored(const PddlState& st)
{
	return (explored(mdp_->get_state(st)).get() != Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
}


dd_node_ptr SymbolicHeuristicAlgorithm::tips(const dd_node_ptr& states)
{
	return dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), tip_states_.get(), states.get()));
}


bool SymbolicHeuristicAlgorithm::is_tip(const PddlState& st)
{
	return (tips(mdp_->get_state(st)).get() != Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));
}


void SymbolicHeuristicAlgorithm::initialize(dd_node_ptr& states)
{
	ScopedConnectionBDD states_scoped_connection(states);
	dd_node_ptr states_to_expand(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), states.get(), mdp_->get_unexpanded_states().get()));
	mdp_->expand(states_to_expand);
	dd_node_ptr next_states(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));

	for (SymbolicMDP::transitions_map_t::const_iterator Itr = mdp_->get_transitions().begin() ; Itr != mdp_->get_transitions().end() ; ++Itr)
	{
		dd_node_ptr deterministic_transitions(Cudd_addBddPattern(dd_node_ptr::get_cudd_manager(), Itr->second.get_probabilities().get()));
		dd_node_ptr action_next_states(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), deterministic_transitions.get(), states.get()));
		action_next_states = dd_node_ptr(Cudd_bddExistAbstract(dd_node_ptr::get_cudd_manager(), action_next_states.get(), mdp_->get_unprimed_variables_bdd_cube().get()));
		action_next_states = dd_node_ptr(Cudd_bddPermute(dd_node_ptr::get_cudd_manager(), action_next_states.get(), mdp_->get_primed_to_unprimed_permutation()));
		next_states = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), next_states.get(), action_next_states.get()));
	}

	dd_node_ptr and_dd(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), explored_states_.get(), next_states.get()));
	dd_node_ptr xor_dd(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), next_states.get(), and_dd.get()));
	ScopedConnectionBDD xor_scoped_connection(xor_dd);
	heuristic_->compute(xor_dd);
	tip_states_ = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), tip_states_.get(), xor_dd.get()));
	tip_states_ = dd_node_ptr(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), tip_states_.get(), states.get()));
	explored_states_ = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), explored_states_.get(), xor_dd.get()));
}


dd_node_ptr SymbolicHeuristicAlgorithm::compute_policy_deterministic_transitions(const dd_node_ptr& states) const
{
	dd_node_ptr policy_deterministic_transitions(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));

	for (std::map<CUDD_VALUE_TYPE, const Action*>::const_iterator Iaction = mdp_->get_index_to_action().begin() ; Iaction != mdp_->get_index_to_action().end() ; ++Iaction)
	{
		dd_node_ptr action_nb_dd(Cudd_addConst(dd_node_ptr::get_cudd_manager(), Iaction->first));
		dd_node_ptr action_deterministic_transitions(Cudd_addBddPattern(dd_node_ptr::get_cudd_manager(), mdp_->get_transitions().find(Iaction->second)->second.get_probabilities().get()));
		action_deterministic_transitions = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), action_deterministic_transitions.get(), states.get()));
		dd_node_ptr action_policy(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addAgreement, mdp_->get_policy().get(), action_nb_dd.get()));
		action_policy = dd_node_ptr(Cudd_addBddPattern(dd_node_ptr::get_cudd_manager(), action_policy.get()));
		action_deterministic_transitions = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), action_deterministic_transitions.get(), action_policy.get()));
		policy_deterministic_transitions = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), policy_deterministic_transitions.get(), action_deterministic_transitions.get()));
	}

	return policy_deterministic_transitions;
}
