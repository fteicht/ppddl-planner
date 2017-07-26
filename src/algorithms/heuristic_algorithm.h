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

#ifndef HEURISTIC_ALGORITHM_H_
#define HEURISTIC_ALGORITHM_H_

#include "base_algorithm.h"
#include "heuristics/base_heuristic.h"
#include "planners/base_planner.h"

class HeuristicAlgorithm : public virtual BaseAlgorithm
{
protected :
	heuristic_t heuristic_type_;

public :
	HeuristicAlgorithm(const Problem& pb, double epsilon, double discount_factor, heuristic_t heuristic_type); // throws;
	virtual ~HeuristicAlgorithm() {}

	virtual const BaseHeuristic& get_heuristic() const =0;
	inline heuristic_t get_heuristic_type() const {return heuristic_type_;}

	/* Returns TRUE if the policy (even not complete) is defined in the given state */
	virtual bool policy_defined(const PddlState& st);
};


template <class Tstate>
class GraphHeuristicAlgorithm : public HeuristicAlgorithm, public GraphAlgorithm<Tstate>
{
public :
	typedef std::set<Tstate*> states_set_t;

protected :
	GraphHeuristic<Tstate>* heuristic_;
	Tstate* initial_state_;

	void initialize(Tstate& st);
	inline void _delete_transitions_(const typename GraphMDP<Tstate>::actions_map_t* transitions);

public :
	GraphHeuristicAlgorithm(const Problem& pb, double epsilon, double discount_factor, unsigned int plan_length, heuristic_t heuristic_type,
							determinization_t determinization_type, deterministic_planner_t deterministic_planner_type); // throws
	virtual ~GraphHeuristicAlgorithm();

	virtual const BaseHeuristic& get_heuristic() const {return (*heuristic_);}

	/* Initialize the optimization process with a given intial state */
	virtual void solve_initialize(const PddlState& st);
};


class SymbolicHeuristicAlgorithm : public HeuristicAlgorithm, public SymbolicAlgorithm
{
protected :
	SymbolicHeuristic* heuristic_;
	dd_node_ptr initial_state_;
	dd_node_ptr explored_states_;
	dd_node_ptr tip_states_;

	void initialize(dd_node_ptr& states);
	dd_node_ptr compute_policy_deterministic_transitions(const dd_node_ptr& states) const;

public :
	SymbolicHeuristicAlgorithm(const Problem& pb, double epsilon, double discount_factor, unsigned int plan_length, heuristic_t heuristic_type,
								determinization_t determinization_type, deterministic_planner_t deterministic_planner_type); // throws
	~SymbolicHeuristicAlgorithm();

	virtual const BaseHeuristic& get_heuristic() const {return (*heuristic_);}

	/* Returns true if the given state has been explored */
	virtual bool explored(const PddlState& st);

	/* Returns the set of explored states among the given set of states */
	dd_node_ptr explored(const dd_node_ptr& st);

	/* Returns true if the given state is a tip state */
	virtual bool is_tip(const PddlState& st);

	/* Returns the set of tip states among the given set of states */
	dd_node_ptr tips(const dd_node_ptr& st);

	/* Initialize the optimization process with a given intial state */
	virtual void solve_initialize(const PddlState& st);
};

#endif /*HEURISTIC_ALGORITHM_H_*/
