/* 
   ppddl-planner - client for IPPC'08

   Copyright (C) 2012 Florent Teichteil-Koenigsbuch and Guillaume Infantes and Ugur Kuter

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

#ifndef SYMBOLIC_GCO_H_
#define SYMBOLIC_GCO_H_

#include "base_algorithm.h"
#include "models/symbolic_mdp.h"

// Goal probability and goal cost optimizations
class SymbolicGCO : public GoalStatisticsAlgorithm, public SymbolicAlgorithm
{
private :
    dd_node_ptr record_probabilities_;
    dd_node_ptr record_values_;
    bool probability_converged_;
	
	dd_node_ptr restricted_states_;
	dd_node_ptr pre_probability_function_;
	dd_node_ptr post_probability_function_;
	
	dd_node_ptr compute_action_probability(const dd_node_ptr& primed_probability_function, const SymbolicTransition& transitions) const;
	dd_node_ptr compute_action_cost(const dd_node_ptr& primed_value_function, const SymbolicTransition& transitions) const;
	void compute_cost_policy(dd_node_ptr& policy, dd_node_ptr& value_function, const dd_node_ptr& primed_value_function, const SymbolicMDP::transitions_map_t& transitions) const;
    
public :
	/* Constructs a GCO algorithm */
	SymbolicGCO(const Problem& pb, double epsilon); // throws
	
	/* Destructs this algorithm */
	virtual ~SymbolicGCO() {}
	
	/* Returns TRUE if the policy (even not complete) is defined in the given state */
	virtual bool policy_defined(const PddlState& st);
	
	/* Initialize the optimization process with a given intial state */
	virtual void solve_initialize(const PddlState& st);
	
	/* Progress in the optimization: perform one step of the main algorithm's loop */
	virtual void solve_progress();
	
	/* Return TRUE if the algorithm has converged */
	virtual bool has_converged();
    
    virtual double goal_probability(const PddlState& st) const;
    
    virtual double goal_cost(const PddlState& st) const;
};

#endif /*SYMBOLIC_GCO_H_*/
