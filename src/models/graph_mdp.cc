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
#include <cstdlib>
#include <sstream>

#include "graph_mdp.h"


// CLASS GraphState

GraphState::GraphState(const PddlState& pddl_state)
: pddl_state_(&pddl_state), value_(0), policy_(NULL), expanded_(false)
{
}


const Action& GraphState::get_policy() const
{
	if (!policy_)
	{
		std::ostringstream oss_st;
		oss_st << "state " << (*pddl_state_) << ": uninitialized policy";
		throw BaseException(oss_st.str(), "GraphState::get_policy");
	}
	
	return (*policy_);
}


void GraphState::print(std::ostream& os) const
{
	os << "state: " << (*pddl_state_) << std::endl;
}


std::ostream& operator<<(std::ostream& os, const GraphState& st)
{
	st.print(os);
	return os;
}


// Class GraphStateProbability

GraphStateProbability::GraphStateProbability(const PddlState& pddl_state)
: GraphState(pddl_state), probability_(0.0)
{
}


// Class GraphStateVariance

GraphStateVariance::GraphStateVariance(const PddlState& pddl_state)
: GraphState(pddl_state), variance_(0.0)
{
}


// Class GraphStateSolveBit

GraphStateSolveBit::GraphStateSolveBit(const PddlState& pddl_state)
: GraphState(pddl_state), solved_(false)
{
}


// Class GraphStateConnection

GraphStateConnection::GraphStateConnection(const PddlState& pddl_state)
: GraphState(pddl_state), idx_(INFINITE_INDEX), low_(INFINITE_INDEX)
{
}


// Class GraphStateSolveBitConnection
GraphStateSolveBitConnection::GraphStateSolveBitConnection(const PddlState& pddl_state)
: GraphState(pddl_state), GraphStateSolveBit(pddl_state), GraphStateConnection(pddl_state)
{
}


// CLASS GraphTransition

template <class Tstate>
GraphTransition<Tstate>::GraphTransition(double probability, double reward, Tstate& outcome) // throws
try : probability_(probability), reward_(reward), outcome_(&outcome)
{
	if ((probability_ < 0.0) || (probability_ > 1.0))
		throw BaseException("probability must be in range [0;1]", "GraphTransition::GraphTransition");
}
catch (...)
{
	throw;
}


template <class Tstate>
std::ostream& operator<<(std::ostream& os, const GraphTransition<Tstate>& transition)
{
	os << "p=[" << transition.get_probability() << "] r=[" << transition.get_reward() << "] s'=[" << (transition.get_outcome().get_pddl_state()) << "]" << std::endl;
	return os;
}


// Template instantiations
INSTANTIATE_STATE_TEMPLATE(GraphTransition)


// CLASS _GraphStateWT_

template <class Tstate>
_GraphStateWT_<Tstate>::~_GraphStateWT_()
{
	for (typename actions_map_t::iterator Iac = transitions_.begin() ; Iac != transitions_.end() ; ++Iac)
	{
		for (typename std::list<GraphTransition<Tstate>*>::iterator Itr = Iac->second.begin() ; Itr != Iac->second.end() ; ++Itr)
			delete (*Itr);
	}
}


template <class Tstate>
void _GraphStateWT_<Tstate>::print_transitions(std::ostream& os) const
{
	os << "    transitions:" << std::endl;
	
	for (typename actions_map_t::const_iterator Iac = transitions_.begin() ; Iac != transitions_.end() ; ++Iac)
	{
		os << "        action: " << (*(Iac->first)) << std::endl;
		
		for (typename std::list<GraphTransition<Tstate>*>::const_iterator Itr = Iac->second.begin() ; Itr != Iac->second.end() ; ++Itr)
			os << "            " << (**Itr);
	}
}


// Template instantiations
template class _GraphStateWT_<GraphStateWT<GraphState> >;
template class _GraphStateWT_<GraphStateWT<GraphStateProbability> >;
template class _GraphStateWT_<GraphStateWT<GraphStateVariance> >;
template class _GraphStateWT_<GraphStateWT<GraphStateSolveBit> >;
template class _GraphStateWT_<GraphStateWT<GraphStateConnection> >;
template class _GraphStateWT_<GraphStateWT<GraphStateSolveBitConnection> >;
template class _GraphStateWT_<GraphStateWTA<GraphState> >;
template class _GraphStateWT_<GraphStateWTA<GraphStateProbability> >;
template class _GraphStateWT_<GraphStateWTA<GraphStateVariance> >;
template class _GraphStateWT_<GraphStateWTA<GraphStateSolveBit> >;
template class _GraphStateWT_<GraphStateWTA<GraphStateConnection> >;
template class _GraphStateWT_<GraphStateWTA<GraphStateSolveBitConnection> >;


// CLASS _GraphStateWA_

template <class Tstate>
void _GraphStateWA_<Tstate>::print_ancestors(std::ostream& os) const
{
	os << "    ancestors:" << std::endl;
	
	for (typename ancestors_map_t::const_iterator Ian = ancestors_.begin() ; Ian != ancestors_.end() ; ++Ian)
	{
		os << "        state: " << (Ian->first->get_pddl_state()) << std::endl;
		
		for (typename std::map<const Action*, const GraphTransition<Tstate>*>::const_iterator Imp = Ian->second.begin() ; Imp != Ian->second.end() ; ++Imp)
			os << "            a={" << (*(Imp->first)) << "} t={" << (*(Imp->second)) << "}" << std::endl;
	}
}


// Template instantiations
template class _GraphStateWA_<GraphStateWA<GraphState> >;
template class _GraphStateWA_<GraphStateWA<GraphStateProbability> >;
template class _GraphStateWA_<GraphStateWA<GraphStateVariance> >;
template class _GraphStateWA_<GraphStateWA<GraphStateSolveBit> >;
template class _GraphStateWA_<GraphStateWA<GraphStateConnection> >;
template class _GraphStateWA_<GraphStateWA<GraphStateSolveBitConnection> >;
template class _GraphStateWA_<GraphStateWTA<GraphState> >;
template class _GraphStateWA_<GraphStateWTA<GraphStateProbability> >;
template class _GraphStateWA_<GraphStateWTA<GraphStateVariance> >;
template class _GraphStateWA_<GraphStateWTA<GraphStateSolveBit> >;
template class _GraphStateWA_<GraphStateWTA<GraphStateConnection> >;
template class _GraphStateWA_<GraphStateWTA<GraphStateSolveBitConnection> >;


// CLASS GraphStateWT

template <>
GraphStateWT<GraphState>::GraphStateWT(const PddlState& pddl_state)
: GraphState(pddl_state), _GraphStateWT_<GraphStateWT<GraphState> >()
{
}


template <class Tstate>
GraphStateWT<Tstate>::GraphStateWT(const PddlState& pddl_state)
: GraphState(pddl_state), Tstate(pddl_state), _GraphStateWT_<GraphStateWT<Tstate> >()
{
}


template <class Tstate>
void GraphStateWT<Tstate>::print(std::ostream& os) const
{
	Tstate::print(os);
	this->print_transitions(os);
}


// Template instantiations
template class GraphStateWT<GraphState>;
template class GraphStateWT<GraphStateProbability>;
template class GraphStateWT<GraphStateVariance>;
template class GraphStateWT<GraphStateSolveBit>;
template class GraphStateWT<GraphStateConnection>;
template class GraphStateWT<GraphStateSolveBitConnection>;


// CLASS GraphStateWA

template <>
GraphStateWA<GraphState>::GraphStateWA(const PddlState& pddl_state)
: GraphState(pddl_state), _GraphStateWA_<GraphStateWA<GraphState> >()
{
}


template <class Tstate>
GraphStateWA<Tstate>::GraphStateWA(const PddlState& pddl_state)
: GraphState(pddl_state), Tstate(pddl_state), _GraphStateWA_<GraphStateWA<Tstate> >()
{
}


template <class Tstate>
void GraphStateWA<Tstate>::print(std::ostream& os) const
{
	Tstate::print(os);
	this->print_ancestors(os);
}


// Template instantiations
template class GraphStateWA<GraphState>;
template class GraphStateWA<GraphStateProbability>;
template class GraphStateWA<GraphStateVariance>;
template class GraphStateWA<GraphStateSolveBit>;
template class GraphStateWA<GraphStateConnection>;
template class GraphStateWA<GraphStateSolveBitConnection>;


// CLASS GraphStateWTA

template <>
GraphStateWTA<GraphState>::GraphStateWTA(const PddlState& pddl_state)
: GraphState(pddl_state), _GraphStateWT_<GraphStateWTA<GraphState> >(), _GraphStateWA_<GraphStateWTA<GraphState> >()
{
}


template <class Tstate>
GraphStateWTA<Tstate>::GraphStateWTA(const PddlState& pddl_state)
: GraphState(pddl_state), Tstate(pddl_state), _GraphStateWT_<GraphStateWTA<Tstate> >(), _GraphStateWA_<GraphStateWTA<Tstate> >()
{
}


template <class Tstate>
void GraphStateWTA<Tstate>::print(std::ostream& os) const
{
	Tstate::print(os);
	this->print_transitions(os);
	this->print_ancestors(os);
}


// Template instantiations
template class GraphStateWTA<GraphState>;
template class GraphStateWTA<GraphStateProbability>;
template class GraphStateWTA<GraphStateVariance>;
template class GraphStateWTA<GraphStateSolveBit>;
template class GraphStateWTA<GraphStateConnection>;
template class GraphStateWTA<GraphStateSolveBitConnection>;


// CLASS GraphMDP

template <class Tstate>
GraphMDP<Tstate>::~GraphMDP()
{
	for (typename states_map_t::const_iterator Ist = states_.begin() ; Ist != states_.end() ; ++Ist)
	{
		delete (Ist->second);
		delete (Ist->first);
	}
}


template <class Tstate>
Tstate& GraphMDP<Tstate>::_add_state_(PddlState* key, bool goal)
{
	Tstate* state = new Tstate(*key);
	std::pair<typename states_map_t::iterator, bool> new_state = states_.insert(std::make_pair(key, state));
	
	if (!new_state.second)
	{
		delete state;
		delete key;
		state = new_state.first->second;
	}
	else
	{
		new_states_.push_back(new_state.first);
		
		if (goal)
			goal_states_.insert(state);
	}
	
	return (*state);
}


template <class Tstate>
Tstate& GraphMDP<Tstate>::add_state(const PddlState& st)
{
	PddlState* key = st.copy();
	const Problem& problem = tb_.get_problem();
	const StateFormula& goal_formula = problem.goal();

	if (goal_formula.holds(problem.terms(), key->atoms(), key->values()))
		return _add_state_(key, true);
	else
		return _add_state_(key, false);
}


template <class Tstate>
void GraphMDP<Tstate>::remove_state(const PddlState& st)
{
	typename states_map_t::iterator Ist = states_.find(&st);
	
	if (Ist != states_.end())
		states_.erase(Ist);
}


template <class Tstate>
const Tstate& GraphMDP<Tstate>::get_state(const PddlState& st) const
{
	typename states_map_t::const_iterator Ist = states_.find(&st);
	
	if (Ist == states_.end())
		throw BaseException("unknown state", "GraphMDP::get_state");
	
	return (*(Ist->second));
}


template <class Tstate>
Tstate& GraphMDP<Tstate>::_state_(const PddlState& st)
{
	typename states_map_t::iterator Ist = states_.find(&st);
	
	if (Ist == states_.end())
		throw BaseException("unknown state", "GraphMDP::_state_");
	
	return (*(Ist->second));
}


template <class Tstate>
const StateFormula& GraphMDP<Tstate>::state_formula(const states_set_t& states) const
{
	const StateFormula* res = &StateFormula::FALSE;
	
	for (typename states_set_t::const_iterator Ist = states.begin() ; Ist != states.end() ; ++Ist)
		res = &((*res) || ((*Ist)->get_pddl_state().state_formula()));
	
	return (*res);
}


template <class Tstate>
std::pair<Tstate*, double> GraphMDP<Tstate>::random_transition(Tstate& st, const Action& action)
{
	new_states_.clear();
	const typename GraphMDP<Tstate>::actions_map_t* actions = _random_transition_(st);
	
	if (!actions) // pick from PDDL model
	{
		double reward;
		bool goal;
		PddlState* outcome_key = new PddlState();
		tb_.random_transition(st.get_pddl_state(), action, *outcome_key, reward, goal);
		Tstate& outcome = _add_state_(outcome_key, goal);
		return std::make_pair(&outcome, reward);
	}
	else // pick from existing transitions
	{
		typename actions_map_t::const_iterator Iac = actions->find(&action);
		
		if (Iac == actions->end())
			throw BaseException("unknown action", "GraphMDP::random_transition");
		
		const std::list<GraphTransition<Tstate>*>& effects = Iac->second;
		
		if (effects.empty())
			throw BaseException("no effects for the given action", "GraphMDP::random_transition");
		
		double prob = std::rand() / static_cast<double>(RAND_MAX);
		double cumulative_prob = 0.0;
		
		for (typename std::list<GraphTransition<Tstate>*>::const_iterator Ieff = effects.begin() ; Ieff != effects.end() ; ++Ieff)
		{
			cumulative_prob += (*Ieff)->get_probability();
			
			if (prob <= cumulative_prob)
				return std::make_pair(&((*Ieff)->_outcome_()), (*Ieff)->get_reward());
		}
	}
	
	throw BaseException("this line should never be reached", "GraphMDP::random_transition");
}


template <class Tstate>
void GraphMDP<Tstate>::transition_blackbox(const Action& action, double probability, double reward, PddlState* outcome, bool goal)
{
	Tstate& outcome_st = _add_state_(outcome, goal);
	GraphTransition<Tstate>* transition = new GraphTransition<Tstate>(probability, reward, outcome_st);
	(*actions_blackbox_)[&action].push_back(transition);
	_ancestors_(*state_blackbox_, action, *transition);
}


template <class Tstate>
const typename GraphMDP<Tstate>::actions_map_t* GraphMDP<Tstate>::transitions(Tstate& st)
{
	new_states_.clear();
	actions_blackbox_ = _transitions_(st);
	state_blackbox_ = &st;
	
	if (actions_blackbox_->empty())
	{
		tb_.transitions(st.get_pddl_state(), *this);
		st.expand();
	}
	
	return actions_blackbox_;
}


#define INSTANTIATE_GRAPH_MDP_INTERNAL_GS(StateT) \
template <> \
const GraphMDP<StateT>::actions_map_t* GraphMDP<StateT>::_random_transition_(StateT& st) \
{ \
	return NULL; \
} \
template <>  \
GraphMDP<StateT>::actions_map_t* GraphMDP<StateT>::_transitions_(StateT& st) \
{ \
	return (new actions_map_t()); \
} \
template <> \
void GraphMDP<StateT>::_ancestors_(StateT& st, const Action& ac, GraphTransition<StateT>& transition) \
{ \
}

INSTANTIATE_GRAPH_MDP_INTERNAL_GS(GraphState)
INSTANTIATE_GRAPH_MDP_INTERNAL_GS(GraphStateProbability)
INSTANTIATE_GRAPH_MDP_INTERNAL_GS(GraphStateVariance)
INSTANTIATE_GRAPH_MDP_INTERNAL_GS(GraphStateSolveBit)
INSTANTIATE_GRAPH_MDP_INTERNAL_GS(GraphStateConnection)
INSTANTIATE_GRAPH_MDP_INTERNAL_GS(GraphStateSolveBitConnection)


#define INSTANTIATE_GRAPH_MDP_INTERNAL_WA(StateT) \
template <> \
const GraphMDP<GraphStateWA<StateT> >::actions_map_t* GraphMDP<GraphStateWA<StateT> >::_random_transition_(GraphStateWA<StateT>& st) \
{ \
	return NULL; \
} \
template <> \
GraphMDP<GraphStateWA<StateT> >::actions_map_t* GraphMDP<GraphStateWA<StateT> >::_transitions_(GraphStateWA<StateT>& st) \
{ \
	return (new actions_map_t()); \
} \
template <> \
void GraphMDP<GraphStateWA<StateT> >::_ancestors_(GraphStateWA<StateT>& st, const Action& ac, GraphTransition<GraphStateWA<StateT> >& transition) \
{ \
	transition._outcome_()._ancestors_()[&st][&ac] = &transition; \
}

INSTANTIATE_GRAPH_MDP_INTERNAL_WA(GraphState)
INSTANTIATE_GRAPH_MDP_INTERNAL_WA(GraphStateProbability)
INSTANTIATE_GRAPH_MDP_INTERNAL_WA(GraphStateVariance)
INSTANTIATE_GRAPH_MDP_INTERNAL_WA(GraphStateSolveBit)
INSTANTIATE_GRAPH_MDP_INTERNAL_WA(GraphStateConnection)
INSTANTIATE_GRAPH_MDP_INTERNAL_WA(GraphStateSolveBitConnection)


#define INSTANTIATE_GRAPH_MDP_INTERNAL_WT(StateT) \
template <> \
const GraphMDP<GraphStateWT<StateT> >::actions_map_t* GraphMDP<GraphStateWT<StateT> >::_random_transition_(GraphStateWT<StateT>& st) \
{ \
	if (st.get_transitions().empty()) \
		return NULL; \
	else \
		return (&(st.get_transitions())); \
} \
template <> \
GraphMDP<GraphStateWT<StateT> >::actions_map_t* GraphMDP<GraphStateWT<StateT> >::_transitions_(GraphStateWT<StateT>& st) \
{ \
	return (&(st._transitions_())); \
} \
template <> \
void GraphMDP<GraphStateWT<StateT> >::_ancestors_(GraphStateWT<StateT>& st, const Action& ac, GraphTransition<GraphStateWT<StateT> >& transition) \
{ \
}

INSTANTIATE_GRAPH_MDP_INTERNAL_WT(GraphState)
INSTANTIATE_GRAPH_MDP_INTERNAL_WT(GraphStateProbability)
INSTANTIATE_GRAPH_MDP_INTERNAL_WT(GraphStateVariance)
INSTANTIATE_GRAPH_MDP_INTERNAL_WT(GraphStateSolveBit)
INSTANTIATE_GRAPH_MDP_INTERNAL_WT(GraphStateConnection)
INSTANTIATE_GRAPH_MDP_INTERNAL_WT(GraphStateSolveBitConnection)


#define INSTANTIATE_GRAPH_MDP_INTERNAL_WTA(StateT) \
template <> \
const GraphMDP<GraphStateWTA<StateT> >::actions_map_t* GraphMDP<GraphStateWTA<StateT> >::_random_transition_(GraphStateWTA<StateT>& st) \
{ \
	if (st.get_transitions().empty()) \
		return NULL; \
	else \
		return (&(st.get_transitions())); \
} \
template <> \
GraphMDP<GraphStateWTA<StateT> >::actions_map_t* GraphMDP<GraphStateWTA<StateT> >::_transitions_(GraphStateWTA<StateT>& st) \
{ \
	return (&(st._transitions_())); \
} \
template <> \
void GraphMDP<GraphStateWTA<StateT> >::_ancestors_(GraphStateWTA<StateT>& st, const Action& ac, GraphTransition<GraphStateWTA<StateT> >& transition) \
{ \
	transition._outcome_()._ancestors_()[&st][&ac] = &transition; \
}

INSTANTIATE_GRAPH_MDP_INTERNAL_WTA(GraphState)
INSTANTIATE_GRAPH_MDP_INTERNAL_WTA(GraphStateProbability)
INSTANTIATE_GRAPH_MDP_INTERNAL_WTA(GraphStateVariance)
INSTANTIATE_GRAPH_MDP_INTERNAL_WTA(GraphStateSolveBit)
INSTANTIATE_GRAPH_MDP_INTERNAL_WTA(GraphStateConnection)
INSTANTIATE_GRAPH_MDP_INTERNAL_WTA(GraphStateSolveBitConnection)


template <class Tstate>
const Action& GraphMDP<Tstate>::get_policy(const PddlState& st) const
{
	typename states_map_t::const_iterator Ist = states_.find(&st);
	
	if (Ist == states_.end())
		throw BaseException("unknown state", "GraphMDP::get_policy");
	
	try
	{
		return (Ist->second->get_policy());
	}
	catch (BaseException& error)
	{
		error.push_function_backtrace("GraphMDP::get_policy");
		throw;
	}
}


template <class Tstate>
double GraphMDP<Tstate>::get_value(const PddlState& st) const
{
	typename states_map_t::const_iterator Ist = states_.find(&st);
	
	if (Ist == states_.end())
		throw BaseException("unknown state", "GraphMDP::get_value");
	
	return (Ist->second->get_value());
}


template <class Tstate>
bool GraphMDP<Tstate>::explored(const PddlState& st) const
{
	if (states_.find(&st) != states_.end())
		return true;
	else
		return false;
}


template <class Tstate>
bool GraphMDP<Tstate>::explored(const Tstate& st) const
{
	return explored(st.get_pddl_state());
}


template <class Tstate>
bool GraphMDP<Tstate>::expanded(const Tstate& st) const
{
	if (states_.find(&(st.get_pddl_state())) == states_.end())
		return false;
	else
		return (st.expanded());
}


template <class Tstate>
bool GraphMDP<Tstate>::expanded(const PddlState& st) const
{
	typename states_map_t::const_iterator Ist = states_.find(&st);
	
	if (Ist == states_.end())
		return false;
	else
		return (Ist->second->expanded());
}


template <class Tstate>
bool GraphMDP<Tstate>::goal(const Tstate& st) const
{
	return (goal_states_.find(const_cast<Tstate*>(&st)) != goal_states_.end());
}


template <class Tstate>
bool GraphMDP<Tstate>::goal(const PddlState& st) const
{
	typename states_map_t::const_iterator Ist = states_.find(&st);
	
	if (Ist == states_.end())
	{
		const Problem& problem = tb_.get_problem();
		const StateFormula& goal_formula = problem.goal();
	
		return goal_formula.holds(problem.terms(), st.atoms(), st.values());
	}
	else
		return goal(*(Ist->second));
}


template <class Tstate>
std::string GraphMDP<Tstate>::print_model() const
{
	std::ostringstream model_str;
	model_str << "----------------------------------------" << std::endl;
	model_str << "MDP model:\n" << std::endl;
	
	for (typename states_map_t::const_iterator Ist = states_.begin() ; Ist != states_.end() ; ++Ist)
		Ist->second->print(model_str);
	
	model_str << "----------------------------------------" << std::endl;
	return model_str.str();
}


template <class Tstate>
std::string GraphMDP<Tstate>::print_solution() const
{
	std::ostringstream solution_str;
	solution_str << "----------------------------------------" << std::endl;
	solution_str << "MDP solution:\n" << std::endl;
	
	for (typename states_map_t::const_iterator Ist = states_.begin() ; Ist != states_.end() ; ++Ist)
	{
		solution_str << "state: " << (*(Ist->first)) << std::endl;
		solution_str << "    action: ";
		try
		{
			solution_str << (Ist->second->get_policy()) << std::endl;
		}
		catch (...)
		{
			solution_str << "UNDEFINED" << std::endl;
		}
		solution_str << "    value: " << (Ist->second->get_value()) << std::endl;
	}
	
	solution_str << "----------------------------------------" << std::endl;
	return solution_str.str();
}


// Template instantiations
INSTANTIATE_STATE_TEMPLATE(GraphMDP)
