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

#ifndef VI_CUDD_H_
#define VI_CUDD_H_

#include "base_algorithm.h"
#include "models/symbolic_mdp.h"
#include "commons/base_exceptions.h"

class SymbolicVI : public SymbolicAlgorithm
{
private :
	dd_node_ptr record_values_;
	
public :
	/* Constructs an algorithm */
	SymbolicVI(const Problem& pb, double epsilon, double discount_factor); // throws
	
	/* Destructs this algorithm */
	virtual ~SymbolicVI();
	
	/* Returns TRUE if the policy (even not complete) is defined in the given state */
	virtual bool policy_defined(const PddlState& st);
	
	/* Initialize the optimization process with a given intial state */
	virtual void solve_initialize(const PddlState& st);
	
	/* Progress in the optimization: perform one step of the main algorithm's loop */
	virtual void solve_progress();
	
	/* Return TRUE if the algorithm has converged */
	virtual bool has_converged();
};

#endif /*VI_CUDD_H_*/
