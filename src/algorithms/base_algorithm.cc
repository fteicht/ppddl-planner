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
#include "base_algorithm.h"
#include "commons/base_exceptions.h"
#include "algorithms/graph_gce.h"


// CLASS BaseAlgorithm

BaseAlgorithm::BaseAlgorithm(const Problem& pb, double epsilon, double discount_factor)
try : epsilon_(epsilon), discount_factor_(discount_factor), bellman_error_(0)
{
	if (epsilon_ <= 0)
		throw BaseException("epsilon must be strictly positive", "BaseAlgorithm::BaseAlgorithm");

	if ((discount_factor_ <= 0) || (discount_factor_ > 1))
		throw BaseException("discount factor must be in range ]0;1]", "BaseAlgorithm::BaseAlgorithm");

	tb_ = new TransitionsBlackbox(pb);
}
catch (...)
{
	throw;
}


void BaseAlgorithm::set_epsilon(double epsilon)
{
	if (epsilon <= 0)
		throw BaseException("epsilon must be strictly positive", "BaseAlgorithm::set_epsilon");

	epsilon_ = epsilon;
}


void BaseAlgorithm::set_discount_factor(double discount_factor)
{
	if ((discount_factor <= 0) || (discount_factor > 1))
		throw BaseException("discount factor must be in range ]0;1]", "BaseAlgorithm::BaseAlgorithm");

	discount_factor_ = discount_factor;
}


std::ostream& operator<<(std::ostream& os, const BaseAlgorithm& a)
{
	os << a.get_mdp().print_model() << std::endl;
	os << a.get_mdp().print_solution() << std::endl;
	return os;
}




// CLASS GraphAlgorithm

template <class Tstate>
GraphAlgorithm<Tstate>::GraphAlgorithm(const Problem& pb, double epsilon, double discount_factor)
try : BaseAlgorithm(pb, epsilon, discount_factor)
{
	mdp_ = new GraphMDP<Tstate>(*tb_);
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphAlgorithm::GraphAlgorithm");
	throw error;
}


template <class Tstate>
GraphAlgorithm<Tstate>::~GraphAlgorithm()
{
	delete mdp_;
}


template <class Tstate> template <class states_container_t>
void GraphAlgorithm<Tstate>::optimize(states_container_t& states_subset)
{
	std::list<double> record_values(states_subset.size());

	while (true)
	{
		// Record values
		std::list<double>::iterator Ivl = record_values.begin();

		for (typename states_container_t::const_iterator Ist = states_subset.begin() ; Ist != states_subset.end() ; ++Ist)
		{
			*Ivl = (*Ist)->get_value();
			++Ivl;
		}

		for (typename states_container_t::const_iterator Ist = states_subset.begin() ; Ist != states_subset.end() ; ++Ist)
			improve_state_value(**Ist);

		bellman_error_ = 0.0;
		Ivl = record_values.begin();

		for (typename states_container_t::const_iterator Ist = states_subset.begin() ; Ist != states_subset.end() ; ++Ist)
		{
			bellman_error_ = std::max(bellman_error_, std::fabs(((*Ist)->get_value()) - (*Ivl)));
			++Ivl;
		}

		if (bellman_error_ < (epsilon_))
			break;
	}
}


#define INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_GS(StateT) \
template <> \
void GraphAlgorithm<StateT>::improve_state_value(StateT& st) \
{ \
	const GraphMDP<StateT>::actions_map_t* transitions = mdp_->transitions(st); \
	improve_state_value(st, *transitions); \
	delete transitions; \
}

INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_GS(GraphState)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_GS(GraphStateProbability)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_GS(GraphStateVariance)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_GS(GraphStateSolveBit)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_GS(GraphStateConnection)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_GS(GraphStateSolveBitConnection)


#define INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WT(StateT) \
template <> \
void GraphAlgorithm<GraphStateWT<StateT> >::improve_state_value(GraphStateWT<StateT>& st) \
{ \
	improve_state_value(st, st.get_transitions()); \
}

INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WT(GraphState)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WT(GraphStateProbability)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WT(GraphStateVariance)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WT(GraphStateSolveBit)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WT(GraphStateConnection)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WT(GraphStateSolveBitConnection)


#define INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WA(StateT) \
template <> \
void GraphAlgorithm<GraphStateWA<StateT> >::improve_state_value(GraphStateWA<StateT>& st) \
{ \
	const GraphMDP<GraphStateWA<StateT> >::actions_map_t* transitions = mdp_->transitions(st); \
	improve_state_value(st, *transitions); \
	delete transitions; \
}

INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WA(GraphState)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WA(GraphStateProbability)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WA(GraphStateVariance)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WA(GraphStateSolveBit)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WA(GraphStateConnection)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WA(GraphStateSolveBitConnection)


#define INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WTA(StateT) \
template <> \
void GraphAlgorithm<GraphStateWTA<StateT> >::improve_state_value(GraphStateWTA<StateT>& st) \
{ \
	improve_state_value(st, st.get_transitions()); \
}

INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WTA(GraphState)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WTA(GraphStateProbability)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WTA(GraphStateVariance)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WTA(GraphStateSolveBit)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WTA(GraphStateConnection)
INSTANTIATE_GRAPH_ALGORITHM_IMPROVE_STATE_VALUE_WTA(GraphStateSolveBitConnection)


template <class Tstate>
void GraphAlgorithm<Tstate>::improve_state_value(Tstate& st, const actions_map_t& transitions)
{
	if (transitions.empty())
		return;

	double best_q_value = compute_q_value(transitions.begin()->second);
	const Action* best_action = transitions.begin()->first;

	for (typename actions_map_t::const_iterator Iac = ++(transitions.begin()) ; Iac != transitions.end() ; ++Iac)
	{
		double q_value = compute_q_value(Iac->second);

		if (q_value > best_q_value)
		{
			best_q_value = q_value;
			best_action = Iac->first;
		}
	}

	st.set_value(best_q_value);
	st.set_policy(*best_action);
}


template <class Tstate>
double GraphAlgorithm<Tstate>::compute_q_value(const transitions_list_t& effects)
{
	double q_value = 0.0;

	for (typename transitions_list_t::const_iterator Ieff = effects.begin() ; Ieff != effects.end() ; ++Ieff)
		q_value += ((*Ieff)->get_probability()) * (((*Ieff)->get_reward()) + (discount_factor_ * ((*Ieff)->get_outcome().get_value())));

	return q_value;
}


template <class Tstate>
std::pair<double, double> GraphAlgorithm<Tstate>::compute_goal_statistics(algorithm_t alg, const PddlState& st)
{
    GoalStatisticsAlgorithm* algO = NULL;

    switch (alg)
    {
        case ALGORITHM_GCE :
        {
            algO = new GraphGCE<Tstate>(this->tb_->get_problem(), this->epsilon_, *this);
            break;
        }

        default:
            throw BaseException("no goal statistics evaluation algorithm", "GraphAlgorithm::compute_goal_statistics");
    }

    algO->solve_initialize(st);

    do
    {
        algO->solve_progress();
    }
    while (!(algO->has_converged()));

    return std::make_pair(algO->goal_probability(st), algO->goal_cost(st));
}


template <class Tstate>
unsigned int GraphAlgorithm<Tstate>::reachable_state_space_size() const
{
    return mdp_->get_states().size();
}


// Explicit instantiations
INSTANTIATE_STATE_TEMPLATE(GraphAlgorithm)

#define INSTANTIATE_GRAPH_ALGORITHM_OPTIMIZE(StateT) \
	template void GraphAlgorithm<StateT>::optimize(GraphAlgorithm<StateT>::states_set_t&); \
	template void GraphAlgorithm<StateT>::optimize(GraphAlgorithm<StateT>::states_list_t&);

INSTANTIATE_STATE_TEMPLATE_MACRO(INSTANTIATE_GRAPH_ALGORITHM_OPTIMIZE)




// CLASS SymbolicAlgorithm

SymbolicAlgorithm::SymbolicAlgorithm(const Problem& pb, double epsilon, double discount_factor)
try : BaseAlgorithm(pb, epsilon, discount_factor)
{
	mdp_ = new SymbolicMDP(*tb_);
	discount_factor_dd_ = dd_node_ptr(Cudd_addConst(dd_node_ptr::get_cudd_manager(), discount_factor));
}
catch (BaseException& error)
{
	error.push_function_backtrace("SymbolicAlgorithm::SymbolicAlgorithm");
	throw error;
}


SymbolicAlgorithm::~SymbolicAlgorithm()
{
	discount_factor_dd_.clear();
	delete mdp_;
}


void SymbolicAlgorithm::optimize(const dd_node_ptr& states_subset)
{
	dd_node_ptr states_fringe;
	compute_fringe(states_subset, states_fringe);
	SymbolicMDP::transitions_map_t local_transitions;
	compute_local_transitions(states_subset, local_transitions);
	dd_node_ptr states_subset_add(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), states_subset.get()));
	dd_node_ptr local_value_function(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, mdp_->get_value_function().get(), states_subset_add.get()));;
	dd_node_ptr states_fringe_add(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), states_fringe.get()));
	dd_node_ptr value_function_fringe(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, mdp_->get_value_function().get(), states_fringe_add.get()));

	if (local_transitions.empty())
		return;

	while (true)
	{
		dd_node_ptr record_values;
		record_values.copy(local_value_function);

		dd_node_ptr primed_local_value_function(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, local_value_function.get(), value_function_fringe.get()));
		primed_local_value_function = dd_node_ptr(Cudd_addPermute(dd_node_ptr::get_cudd_manager(), primed_local_value_function.get(), mdp_->get_unprimed_to_primed_permutation()));
		primed_local_value_function = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, discount_factor_dd_.get(), primed_local_value_function.get()));
		local_value_function = compute_action_value(primed_local_value_function, local_transitions.begin()->second);

		for (SymbolicMDP::transitions_map_t::const_iterator Itr = ++(local_transitions.begin()) ; Itr != local_transitions.end() ; ++Itr)
		{
			dd_node_ptr action_value = compute_action_value(primed_local_value_function, Itr->second);
			local_value_function = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addMaximum, local_value_function.get(), action_value.get()));
		}

		compute_bellman_error(record_values, local_value_function);

		if (bellman_error_ < epsilon_)
			break;
	}

	// Computes the local policy
	dd_node_ptr primed_local_value_function(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, local_value_function.get(), value_function_fringe.get()));
	primed_local_value_function = dd_node_ptr(Cudd_addPermute(dd_node_ptr::get_cudd_manager(), primed_local_value_function.get(), mdp_->get_unprimed_to_primed_permutation()));
	dd_node_ptr local_policy;
	compute_policy(local_policy, local_value_function, primed_local_value_function, local_transitions);

	// Updates the global value function and the global policy
	dd_node_ptr states_subset_cmpl(Cudd_Not(states_subset.get()));
	states_subset_cmpl = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), states_subset_cmpl.get()));
	mdp_->_value_function_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, mdp_->get_value_function().get(), states_subset_cmpl.get()));
	mdp_->_value_function_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, mdp_->get_value_function().get(), local_value_function.get()));
	mdp_->_policy_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, mdp_->get_policy().get(), states_subset_cmpl.get()));
	mdp_->_policy_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, mdp_->get_policy().get(), local_policy.get()));
}


void SymbolicAlgorithm::compute_fringe(const dd_node_ptr& states_subset, dd_node_ptr& states_fringe) const
{
	dd_node_ptr next_states(Cudd_ReadLogicZero(dd_node_ptr::get_cudd_manager()));

	for (SymbolicMDP::transitions_map_t::const_iterator Itr = mdp_->get_transitions().begin() ; Itr != mdp_->get_transitions().end() ; ++Itr)
	{
		dd_node_ptr deterministic_transitions(Cudd_addBddPattern(dd_node_ptr::get_cudd_manager(), Itr->second.get_probabilities().get()));
		dd_node_ptr action_next_states(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), deterministic_transitions.get(), states_subset.get()));
		action_next_states = dd_node_ptr(Cudd_bddExistAbstract(dd_node_ptr::get_cudd_manager(), action_next_states.get(), mdp_->get_unprimed_variables_bdd_cube().get()));
		action_next_states = dd_node_ptr(Cudd_bddPermute(dd_node_ptr::get_cudd_manager(), action_next_states.get(), mdp_->get_primed_to_unprimed_permutation()));
		next_states = dd_node_ptr(Cudd_bddOr(dd_node_ptr::get_cudd_manager(), next_states.get(), action_next_states.get()));
	}

	dd_node_ptr and_dd(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), states_subset.get(), next_states.get()));
	states_fringe = dd_node_ptr(Cudd_bddXor(dd_node_ptr::get_cudd_manager(), and_dd.get(), next_states.get()));
}


void SymbolicAlgorithm::compute_local_transitions(const dd_node_ptr& states_subset, SymbolicMDP::transitions_map_t& local_transitions) const
{
	dd_node_ptr states_subset_unprimed_add(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), states_subset.get()));

	for (SymbolicMDP::transitions_map_t::const_iterator Itr = mdp_->get_transitions().begin() ; Itr != mdp_->get_transitions().end() ; ++Itr)
	{
		SymbolicTransition& transition = local_transitions[Itr->first];
		transition._policy_mask_() = dd_node_ptr(Cudd_bddAnd(dd_node_ptr::get_cudd_manager(), Itr->second.get_policy_mask().get(), states_subset.get()));
		transition._value_mask_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, Itr->second.get_value_mask().get(), states_subset_unprimed_add.get()));
		transition._probabilities_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, Itr->second.get_probabilities().get(), states_subset_unprimed_add.get()));
		transition._rewards_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, Itr->second.get_rewards().get(), states_subset_unprimed_add.get()));
	}
}


dd_node_ptr SymbolicAlgorithm::compute_action_value(const dd_node_ptr& primed_value_function, const SymbolicTransition& transitions) const
{
	dd_node_ptr action_value(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, primed_value_function.get(), transitions.get_rewards().get()));
	action_value = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, action_value.get(), transitions.get_probabilities().get()));
	action_value = dd_node_ptr(Cudd_addExistAbstract(dd_node_ptr::get_cudd_manager(), action_value.get(), mdp_->get_primed_variables_add_cube().get()));
	action_value = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, action_value.get(), transitions.get_value_mask().get()));
	return action_value;
}


void SymbolicAlgorithm::compute_policy(dd_node_ptr& policy, dd_node_ptr& value_function, const dd_node_ptr& primed_value_function, const SymbolicMDP::transitions_map_t& transitions) const
{
	policy = dd_node_ptr(Cudd_ReadZero(dd_node_ptr::get_cudd_manager()));

	if (mdp_->get_index_to_action().empty())
		return;

	dd_node_ptr primed_value_function_df(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, discount_factor_dd_.get(), primed_value_function.get()));
	value_function = compute_action_value(primed_value_function_df, transitions.find(mdp_->get_index_to_action().begin()->second)->second);
	policy = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), transitions.find(mdp_->get_index_to_action().begin()->second)->second.get_policy_mask().get()));
	dd_node_ptr action_dd(Cudd_addConst(dd_node_ptr::get_cudd_manager(), mdp_->get_index_to_action().begin()->first));
	policy = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, policy.get(), action_dd.get()));

	for (std::map<CUDD_VALUE_TYPE, const Action*>::const_iterator Itr = ++(mdp_->get_index_to_action().begin()) ; Itr != mdp_->get_index_to_action().end() ; ++Itr)
	{
		dd_node_ptr action_value = compute_action_value(primed_value_function_df, transitions.find(Itr->second)->second);
		dd_node_ptr vf_before_max = value_function;
		value_function = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addMaximum, vf_before_max.get(), action_value.get()));
		dd_node_ptr improve_set_dd(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addOneZeroMaximum, action_value.get(), vf_before_max.get()));
		dd_node_ptr cmpl_dd(Cudd_addCmpl(dd_node_ptr::get_cudd_manager(), improve_set_dd.get()));

		action_dd = dd_node_ptr(Cudd_addConst(dd_node_ptr::get_cudd_manager(), Itr->first));
		improve_set_dd = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, improve_set_dd.get(), action_dd.get()));
		cmpl_dd = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, cmpl_dd.get(), policy.get()));
		policy = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addPlus, improve_set_dd.get(), cmpl_dd.get()));
	}
}


void SymbolicAlgorithm::compute_bellman_error(const dd_node_ptr& previous_value_function, const dd_node_ptr& current_value_function)
{
	dd_node_ptr diff_dd(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addMinus, current_value_function.get(), previous_value_function.get()));
	dd_node_ptr bound_dd(Cudd_addFindMax(dd_node_ptr::get_cudd_manager(), diff_dd.get()));
	bellman_error_ = std::fabs(Cudd_V(bound_dd.get()));
	bound_dd = dd_node_ptr(Cudd_addFindMin(dd_node_ptr::get_cudd_manager(), diff_dd.get()));
	bellman_error_ = std::max(bellman_error_, std::fabs(Cudd_V(bound_dd.get())));
}


std::pair<double, double> SymbolicAlgorithm::compute_goal_statistics(algorithm_t alg, const PddlState& st)
{
    throw BaseException("no implemented yet", "SymbolicAlgorithm::compute_goal_statistics");
}


unsigned int SymbolicAlgorithm::reachable_state_space_size() const
{
    return Cudd_CountMinterm(mdp_->get_states().get_cudd_manager(), mdp_->get_states().get(), mdp_->get_atom_to_index().size());
}
