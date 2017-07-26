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

#ifndef STEP_DISTANCE_H_
#define STEP_DISTANCE_H_

#include <set>
#include "base_heuristic.h"

class PddlStepDistance : public virtual BaseHeuristic, public TransitionsBlackboxUser
{
private :
	typedef std::set<const PddlState*, PddlStateLess> states_set_comp_t;
	typedef std::set<const PddlState*> states_set_t;
	states_set_comp_t states_;
	states_set_t new_states_;
	bool goal_state_reached_;
	
	virtual void transition_blackbox(const Action& action, double probability, double reward, PddlState* outcome, bool goal);
	void clear_states();
	
public :
	PddlStepDistance(double discount_factor, const TransitionsBlackbox& tb);
	virtual ~PddlStepDistance() {clear_states();}
	
	virtual double value(const PddlState& st);
};


template <class Tstate>
class PddlGraphStepDistance : public GraphHeuristic<Tstate>, public PddlStepDistance
{
public :
	PddlGraphStepDistance(GraphMDP<Tstate>& mdp, double discount_factor, const TransitionsBlackbox& tb);
	virtual ~PddlGraphStepDistance() {}
	
	virtual void compute(Tstate& st);
	virtual double value(const PddlState& st) {return PddlStepDistance::value(st);}
};


template <class Tstate>
class NativeGraphStepDistance : public GraphHeuristic<Tstate>
{
public :
	typedef std::set<Tstate*> states_set_t;
	typedef typename std::map<Tstate*, std::map<const Action*, const GraphTransition<Tstate>*> > ancestors_map_t;
	typedef typename std::map<Tstate*, ancestors_map_t> states_ancestors_map_t;
	
private :
	states_set_t initialized_states_;
	states_ancestors_map_t states_ancestors_; // used only if Tstate == GraphState||GraphStateWT
	
	inline static void _delete_transitions_(const typename GraphMDP<Tstate>::actions_map_t* transitions);
	inline void _add_ancestor_(Tstate& income, Tstate& outcome, const Action& action, const GraphTransition<Tstate>& transition);
	inline ancestors_map_t& _ancestors_(Tstate& st);
	
public :
	NativeGraphStepDistance(GraphMDP<Tstate>& mdp, double discount_factor); // throws
	virtual ~NativeGraphStepDistance() {}
	
	virtual void compute(Tstate& st);
};


class PddlSymbolicStepDistance : public SymbolicHeuristic, public PddlStepDistance
{
public :
	PddlSymbolicStepDistance(SymbolicMDP& mdp, double discount_factor, const TransitionsBlackbox& tb);
	virtual ~PddlSymbolicStepDistance() {}
	
	virtual void compute(const dd_node_ptr& states);
	virtual double value(const PddlState& st) {return PddlStepDistance::value(st);}
};


class NativeSymbolicStepDistance : public SymbolicHeuristic
{
private :
	dd_node_ptr initialized_states_;
	
	void compute_single_state(const dd_node_ptr& st);
	
public :
	NativeSymbolicStepDistance(SymbolicMDP& mdp, double discount_factor); // throws
	virtual ~NativeSymbolicStepDistance();
	
	virtual void compute(const dd_node_ptr& states);
};

#endif /*STEP_DISTANCE_H_*/
