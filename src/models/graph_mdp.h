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

#ifndef GRAPH_MDP_H_
#define GRAPH_MDP_H_

#include <map>
#include <list>
#include <climits>
#include <ostream>
#include <ppddl_planner_mdpsim/states.h>
#include <ppddl_planner_mdpsim/actions.h>

#include "base_mdp.h"
#include "commons/base_exceptions.h"


class GraphState
{
protected :
	const PddlState* pddl_state_;
	double value_;
	const Action* policy_;
	bool expanded_;
	
public :
	inline GraphState(const PddlState& pddl_state);
	virtual inline ~GraphState() {}
	
	inline const PddlState& get_pddl_state() const {return (*pddl_state_);}
	
	inline void set_value(double value) {value_ = value;}
	inline double get_value() const {return value_;}
	
	inline void set_policy(const Action& policy) {policy_ = &policy;}
	const Action& get_policy() const; // throws
	inline bool policy_defined() const {return (policy_ != NULL);}
	
	inline void expand() {expanded_ = true;}
	inline bool expanded() const {return expanded_;}
	
	virtual void print(std::ostream& os) const;
};

std::ostream& operator<<(std::ostream& os, const GraphState& st);


class GraphStateProbability : public virtual GraphState
{
private :
    double probability_;
    
public :
    inline GraphStateProbability(const PddlState& pddl_state);
    virtual ~GraphStateProbability() {}
    
    inline void set_probability(double probability) {probability_ = probability;}
    inline double get_probability() const {return probability_;}
};


class GraphStateVariance : public virtual GraphState
{
private :
	double variance_;
	
public :
	inline GraphStateVariance(const PddlState& pddl_state);
	virtual ~GraphStateVariance() {}
	
	inline void set_variance(double variance) {variance_ = variance;}
	inline double get_variance() const {return variance_;}
};


class GraphStateSolveBit : public virtual GraphState
{
private :
	bool solved_;
	
public :
	inline GraphStateSolveBit(const PddlState& pddl_state);
	virtual ~GraphStateSolveBit() {}
	
	inline void solve() {solved_ = true;}
	inline bool solved() const {return solved_;}
};


class GraphStateConnection : public virtual GraphState
{
private :
	unsigned long int idx_;
	unsigned long int low_;

public :
	inline GraphStateConnection(const PddlState& pddl_state);
	virtual ~GraphStateConnection() {}
	
	inline void set_idx(unsigned long int idx) {idx_ = idx;}
	inline unsigned long int get_idx() const {return idx_;}
	
	inline void set_low(unsigned long int low) {low_ = low;}
	inline unsigned long int get_low() const {return low_;}
	
	static const unsigned long int INFINITE_INDEX = ULONG_MAX;
};


class GraphStateSolveBitConnection : public GraphStateSolveBit, public GraphStateConnection
{
public :
	GraphStateSolveBitConnection(const PddlState& pddl_state);
	virtual ~GraphStateSolveBitConnection() {}
};


template <class Tstate>
class GraphTransition
{
private :
	double probability_;
	double reward_;
	Tstate* outcome_;

public :
	GraphTransition(double probability, double reward, Tstate& outcome); // throws
	inline ~GraphTransition() {}
	
	inline double get_probability() const {return probability_;}
	inline double get_reward() const {return reward_;}
	
	inline const Tstate& get_outcome() const {return (*outcome_);}
	inline Tstate& _outcome_() {return (*outcome_);}
};

template <class Tstate>
std::ostream& operator<<(std::ostream& os, const GraphTransition<Tstate>& transition);


template <class Tstate>
class _GraphStateWT_ // with transitions
{
public :
	typedef typename std::map<const Action*, std::list<GraphTransition<Tstate>*> > actions_map_t;
	
protected :
	actions_map_t transitions_;
	
	void print_transitions(std::ostream& os) const;
	
public :
	inline _GraphStateWT_() {}
	_GraphStateWT_(const actions_map_t& transitions) : transitions_(transitions) {}
	virtual ~_GraphStateWT_();
	
	inline const actions_map_t& get_transitions() const {return transitions_;}
	inline actions_map_t& _transitions_() {return transitions_;}
};


template <class Tstate>
class _GraphStateWA_ // with ancestors
{
public :
	typedef typename std::map<Tstate*, std::map<const Action*, const GraphTransition<Tstate>*> > ancestors_map_t;
	
protected :
	ancestors_map_t ancestors_;
	
	virtual void print_ancestors(std::ostream& os) const;
	
public :
	inline _GraphStateWA_() {}
	_GraphStateWA_(const ancestors_map_t& ancestors) : ancestors_(ancestors) {}
	virtual inline ~_GraphStateWA_() {}
	
	inline const ancestors_map_t& get_ancestors() const {return ancestors_;}
	inline ancestors_map_t& _ancestors_() {return ancestors_;}
};


template <class Tstate = GraphState>
class GraphStateWT : public Tstate, public _GraphStateWT_<GraphStateWT<Tstate> >
{
public :
	typedef typename _GraphStateWT_<GraphStateWT<Tstate> >::actions_map_t actions_map_t;

public :
	GraphStateWT(const PddlState& pddl_state);
	virtual ~GraphStateWT() {}
	
	virtual void print(std::ostream& os) const;
};


template <class Tstate = GraphState>
class GraphStateWA : public Tstate, public _GraphStateWA_<GraphStateWA<Tstate> >
{
public :
	typedef typename _GraphStateWA_<GraphStateWA<Tstate> >::ancestors_map_t ancestors_map_t;
	
public :
	GraphStateWA(const PddlState& pddl_state);
	virtual ~GraphStateWA() {}
	
	virtual void print(std::ostream& os) const;
};


template <class Tstate = GraphState>
class GraphStateWTA : public Tstate, public _GraphStateWT_<GraphStateWTA<Tstate> >, public _GraphStateWA_<GraphStateWTA<Tstate> >
{
public :
	typedef typename _GraphStateWT_<GraphStateWTA<Tstate> >::actions_map_t actions_map_t;
	typedef typename _GraphStateWA_<GraphStateWTA<Tstate> >::ancestors_map_t ancestors_map_t;

public :
	GraphStateWTA(const PddlState& pddl_state);
	virtual ~GraphStateWTA() {}
	
	virtual void print(std::ostream& os) const;
};


template <class Tstate>
class GraphMDP : public BaseMDP
{
public :
	typedef std::map<const PddlState*, Tstate*, PddlStateLess> states_map_t;
	typedef GraphTransition<Tstate> transition_t;
	typedef std::list<transition_t*> transition_list_t;
	typedef std::map<const Action*, transition_list_t> actions_map_t;
	typedef std::set<Tstate*> states_set_t;
	typedef std::list<typename states_map_t::iterator> new_states_list_t;
	
private :
	states_map_t states_;
	states_set_t goal_states_;
	new_states_list_t new_states_;
	
	actions_map_t* actions_blackbox_;
	Tstate* state_blackbox_;
	
	inline const actions_map_t* _random_transition_(Tstate& st);
	inline actions_map_t* _transitions_(Tstate& st);
	inline void _ancestors_(Tstate& st, const Action& ac, GraphTransition<Tstate>& transition);
	virtual void transition_blackbox(const Action& action, double probability, double reward, PddlState* outcome, bool goal);
	
	Tstate& _add_state_(PddlState* key, bool goal);
	
public :
	inline GraphMDP(const TransitionsBlackbox& tb) : BaseMDP(tb) {}
	virtual ~GraphMDP();
	
	inline const states_map_t& get_states() const {return states_;}
	inline states_map_t& _states_() {return states_;}
	inline const states_set_t& get_goal_states() const {return goal_states_;}
	
	Tstate& add_state(const PddlState& st);
	void remove_state(const PddlState& st);
	const Tstate& get_state(const PddlState& st) const;
	Tstate& _state_(const PddlState& st);
	
	/* Returns the state formula of the given set of states */
	const StateFormula& state_formula(const states_set_t& states) const;
	
	/* Returns a random transition (next state and reward transition) for the given state and the given action */
	std::pair<Tstate*, double> random_transition(Tstate& st, const Action& action);
	
	/* Returns all the actions transitions of the given state and generate new states if necessary 
	 * the returned map must be manually deleted if Tstate=GraphState||GraphStateWA */
	const actions_map_t* transitions(Tstate& st);
	
	/* Returns the states created by the last transitions() or random_transition() functions call */
	const new_states_list_t& new_states() const {return new_states_;}
	
	/* Returns the policy of the given state */
	virtual const Action& get_policy(const PddlState& st) const; // throws
	
	/* Returns the value of the given state */
	virtual double get_value(const PddlState& st) const; // throws
	
	/* Returns true if the given state has been explored */
	virtual bool explored(const PddlState& st) const;
	
	/* Returns true if the given state has been explored */
	bool explored(const Tstate& st) const;
	
	/* Returns true if the given state has been expanded */
	virtual bool expanded(const PddlState& st) const;
	
	/* Returns true if the given state has been expanded */
	bool expanded(const Tstate& st) const;
	
	/* Returns true if the given state is a goal (among explored states) */
	virtual bool goal(const PddlState& st) const;
	
	/* Returns true if the given state is a goal (among explored states) */
	bool goal(const Tstate& st) const;
	
	/* Prints the MDP model */
	virtual std::string print_model() const;
	
	/* Prints the MDP solution */
	virtual std::string print_solution() const;
};


#define INSTANTIATE_STATE_TEMPLATE(TemplateClass) \
	template class TemplateClass<GraphState>; \
    template class TemplateClass<GraphStateProbability>; \
	template class TemplateClass<GraphStateVariance>; \
	template class TemplateClass<GraphStateSolveBit>; \
	template class TemplateClass<GraphStateConnection>; \
	template class TemplateClass<GraphStateSolveBitConnection>; \
	template class TemplateClass<GraphStateWT<GraphState> >; \
    template class TemplateClass<GraphStateWT<GraphStateProbability> >; \
	template class TemplateClass<GraphStateWT<GraphStateVariance> >; \
	template class TemplateClass<GraphStateWT<GraphStateSolveBit> >; \
	template class TemplateClass<GraphStateWT<GraphStateConnection> >; \
	template class TemplateClass<GraphStateWT<GraphStateSolveBitConnection> >; \
	template class TemplateClass<GraphStateWA<GraphState> >; \
    template class TemplateClass<GraphStateWA<GraphStateProbability> >; \
	template class TemplateClass<GraphStateWA<GraphStateVariance> >; \
	template class TemplateClass<GraphStateWA<GraphStateSolveBit> >; \
	template class TemplateClass<GraphStateWA<GraphStateConnection> >; \
	template class TemplateClass<GraphStateWA<GraphStateSolveBitConnection> >; \
	template class TemplateClass<GraphStateWTA<GraphState> >; \
    template class TemplateClass<GraphStateWTA<GraphStateProbability> >; \
	template class TemplateClass<GraphStateWTA<GraphStateVariance> >; \
	template class TemplateClass<GraphStateWTA<GraphStateSolveBit> >; \
	template class TemplateClass<GraphStateWTA<GraphStateConnection> >; \
	template class TemplateClass<GraphStateWTA<GraphStateSolveBitConnection> >;

#define INSTANTIATE_STATE_TEMPLATE_PTR(TemplateClass) \
	template class TemplateClass<GraphState*>; \
    template class TemplateClass<GraphStateProbability*>; \
	template class TemplateClass<GraphStateVariance*>; \
	template class TemplateClass<GraphStateSolveBit*>; \
	template class TemplateClass<GraphStateConnection*>; \
	template class TemplateClass<GraphStateSolveBitConnection*>; \
	template class TemplateClass<GraphStateWT<GraphState>*>; \
    template class TemplateClass<GraphStateWT<GraphStateProbability>*>; \
	template class TemplateClass<GraphStateWT<GraphStateVariance>*>; \
	template class TemplateClass<GraphStateWT<GraphStateSolveBit>*>; \
	template class TemplateClass<GraphStateWT<GraphStateConnection>*>; \
	template class TemplateClass<GraphStateWT<GraphStateSolveBitConnection>*>; \
	template class TemplateClass<GraphStateWA<GraphState>*>; \
    template class TemplateClass<GraphStateWA<GraphStateProbability>*>; \
	template class TemplateClass<GraphStateWA<GraphStateVariance>*>; \
	template class TemplateClass<GraphStateWA<GraphStateSolveBit>*>; \
	template class TemplateClass<GraphStateWA<GraphStateConnection>*>; \
	template class TemplateClass<GraphStateWA<GraphStateSolveBitConnection>*>; \
	template class TemplateClass<GraphStateWTA<GraphState>*>; \
    template class TemplateClass<GraphStateWTA<GraphStateProbability>*>; \
	template class TemplateClass<GraphStateWTA<GraphStateVariance>*>; \
	template class TemplateClass<GraphStateWTA<GraphStateSolveBit>*>; \
	template class TemplateClass<GraphStateWTA<GraphStateConnection>*>; \
	template class TemplateClass<GraphStateWTA<GraphStateSolveBitConnection>*>;

#define INSTANTIATE_STATE_TEMPLATE_MACRO(StateMacro) \
	StateMacro(GraphState) \
    StateMacro(GraphStateProbability) \
	StateMacro(GraphStateVariance) \
	StateMacro(GraphStateSolveBit) \
	StateMacro(GraphStateConnection) \
	StateMacro(GraphStateSolveBitConnection) \
	StateMacro(GraphStateWT<GraphState>) \
    StateMacro(GraphStateWT<GraphStateProbability>) \
	StateMacro(GraphStateWT<GraphStateVariance>) \
	StateMacro(GraphStateWT<GraphStateSolveBit>) \
	StateMacro(GraphStateWT<GraphStateConnection>) \
	StateMacro(GraphStateWT<GraphStateSolveBitConnection>) \
	StateMacro(GraphStateWA<GraphState>) \
    StateMacro(GraphStateWA<GraphStateProbability>) \
	StateMacro(GraphStateWA<GraphStateVariance>) \
	StateMacro(GraphStateWA<GraphStateSolveBit>) \
	StateMacro(GraphStateWA<GraphStateConnection>) \
	StateMacro(GraphStateWA<GraphStateSolveBitConnection>) \
	StateMacro(GraphStateWTA<GraphState>) \
    StateMacro(GraphStateWTA<GraphStateProbability>) \
	StateMacro(GraphStateWTA<GraphStateVariance>) \
	StateMacro(GraphStateWTA<GraphStateSolveBit>) \
	StateMacro(GraphStateWTA<GraphStateConnection>) \
	StateMacro(GraphStateWTA<GraphStateSolveBitConnection>)

#endif /*GRAPH_MDP_H_*/
