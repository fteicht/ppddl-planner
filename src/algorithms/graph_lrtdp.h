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

#ifndef GRAPH_LRTDP_H_
#define GRAPH_LRTDP_H_

#include "heuristic_algorithm.h"
#include <stack>
#include <set>


class GraphLRTDP : public GraphHeuristicAlgorithm<GraphStateWT<GraphStateSolveBit> >
{
public :
	typedef std::set<GraphStateWT<GraphStateSolveBit>*> states_set_t;
	typedef std::stack<GraphStateWT<GraphStateSolveBit>*> states_stack_t;
	
private :
	unsigned int max_depth_;
	double record_value_;
	
	bool check_solved(GraphStateWT<GraphStateSolveBit>& st);
	
public :
	GraphLRTDP(const Problem& pb, double epsilon, double discount_factor, heuristic_t heuristic_type,
				determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
				unsigned int max_depth = 100000); // throws
	~GraphLRTDP() {}
	
	/* Initialize the optimization process with a given intial state */
	virtual void solve_initialize(const PddlState& st);
	
	/* Progress in the optimization: perform one step of the main algorithm's loop */
	virtual void solve_progress();
	
	/* Return TRUE if the algorithm has converged */
	virtual bool has_converged();
	
	inline void set_max_depth(unsigned int max_depth) {max_depth_ = max_depth;}
	inline unsigned int get_max_depth() const {return max_depth_;}
};

#endif /*GRAPH_LRTDP_H_*/
