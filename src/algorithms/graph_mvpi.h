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

#ifndef GRAPH_MVPI_H_
#define	GRAPH_MVPI_H_

#include "graph_mvpf.h"
#include "graph_mvho.h"

/**
 * Base class for mean-variance optimization algorithms based on policy iteration
 */
class GraphMVPI
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
	GraphMDP<GraphStateWT<GraphStateVariance> >::states_map_t::const_iterator state_update_begin_;
	
	/**
	 * Record the current mean policy
	 */
	std::list<const Action*> record_actions_;
	
public :
	GraphMVPI(double proportion_updated_states);
	virtual ~GraphMVPI() {}
};

/**
 * Mean-variance Pareto frontier generation using policy iteration
 */
class GraphMVPFPI : public GraphMVPI, public GraphMVPF<GraphStateWT<GraphStateVariance> >
{
private :
	/**
	 * Compute an initial policy whose mean value is the worst of the best one
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
	/* Constructs a MVPFPI algorithm */
	GraphMVPFPI(const Problem& pb, double epsilon, double discount_factor, double proportion_updated_states); // throws
	
	/* Destructs this algorithm */
	virtual ~GraphMVPFPI() {}
};

/**
 * Mean-variance heuristic optimization using policy iteration
 */
class GraphMVHOPI : public GraphMVPI, public GraphMVHO<GraphStateWT<GraphStateVariance> >
{
private :
	/**
	 * Current proportion of states to update
	 */
	double current_proportion_updated_states_;
	
	/**
	 * Compute an initial policy whose mean value is the worst of the best one
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
	/* Constructs a MVPFPI algorithm */
	GraphMVHOPI(const Problem& pb, double minimum_mean, double maximum_variance, double epsilon, double discount_factor, double proportion_updated_states); // throws
	
	/* Destructs this algorithm */
	virtual ~GraphMVHOPI() {}
};

#endif // GRAPH_MVPI_H_
