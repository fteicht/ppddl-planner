/* 
   ppddl-planner - client for IPPC'08

   Copyright (C) 2013 Florent Teichteil-Koenigsbuch and Andrey Kolobov and Caroline Carvalho and Blai Bonet

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

#ifndef GRAPH_MVSSP_H_
#define	GRAPH_MVSSP_H_

#include "graph_mvpf.h"

/**
 * Base class for mean-variance optimization algorithms based on policy iteration for SSPs
 */
class GraphMVSSP
{
protected :
	/**
	 * Proportion of states to update at each policy improvement step
	 */
	double proportion_updated_states_;
	
	/**
	 * Boolean indicatinf whether the policy has changed
	 */
	bool policy_changed_;
	
	/**
	 * State from which policy improvement is currently performed
	 */
	GraphMDP<GraphStateWTA<GraphStateVariance> >::states_map_t::const_iterator state_update_begin_;
	
	/**
	 * Record the current mean policy
	 */
	std::list<const Action*> record_actions_;
	
public :
	GraphMVSSP(double proportion_updated_states);
	virtual ~GraphMVSSP() {}
};

/**
 * Mean-variance Pareto frontier generation using policy iteration for SSPs
 */
class GraphMVPFSSP : public GraphMVSSP, public GraphMVPF<GraphStateWTA<GraphStateVariance> >
{
private :
	/**
	 * Compute an initial proper policy
     */
	virtual void initial_mean_level_policy();
	
	/**
	 * Generate a policy on the next mean level
     */
	virtual void next_mean_level_policy();
	
	/**
	 * Test whether the current mean level is the last one
     */
	virtual bool last_mean_level() const;
	
public :
	/* Constructs a MVPFSSP algorithm */
	GraphMVPFSSP(const Problem& pb, double epsilon, double proportion_updated_states); // throws
	
	/* Destructs this algorithm */
	virtual ~GraphMVPFSSP() {}
};

#endif // GRAPH_MVSSP_H_
