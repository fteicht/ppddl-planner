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

#ifndef GRAPH_GCE_H_
#define GRAPH_GCE_H_

#include "base_algorithm.h"
#include "models/graph_mdp.h"

// Goal probability and goal cost evaluations
template <class Tstate>
class GraphGCE : public GoalStatisticsAlgorithm, public GraphAlgorithm<Tstate>
{
private :
    struct MyValues
    {
        double previous_goal_probability_value_;
        double current_goal_probability_value_;
        double previous_goal_cost_value_;
        double current_goal_cost_value_;
        MyValues() : previous_goal_probability_value_(0.0),
                     current_goal_probability_value_(0.0),
                     previous_goal_cost_value_(0.0),
                     current_goal_cost_value_(0.0) {}
    };
    
    typedef std::map<const Tstate*, MyValues> MyValuesMap;
    MyValuesMap values_map_;
    
    GraphAlgorithm<Tstate>& algorithm_;
    
    inline const typename GraphMDP<Tstate>::actions_map_t* _transitions_(Tstate& st);
    inline void _delete_transitions_(const typename GraphMDP<Tstate>::actions_map_t* tr);
	
public :
	/* Constructs a GCE algorithm */
	GraphGCE(const Problem& pb, double epsilon, GraphAlgorithm<Tstate>& algorithm); // throws
	
	/* Destructs this algorithm */
	virtual ~GraphGCE() {}
	
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

#endif /*GRAPH_GCE_H_*/
