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

#ifndef GRAPH_MVAS_H_
#define	GRAPH_MVAS_H_

#include "graph_mvpf.h"
#include "graph_mvho.h"

/**
 * Base class for mean-variance optimization algorithms based on action sets
 */
class GraphMVAS
{
protected :
	/**
	 * Manages action sets for a given state
	 */
	struct ActionSetFactory
	{
		bool state_converged_;
		std::list<GraphMDP<GraphStateWT<GraphStateVariance> >::actions_map_t::const_iterator> policy_set_;
		std::vector<GraphMDP<GraphStateWT<GraphStateVariance> >::actions_map_t::const_iterator> candidate_set_;
	};
	
	/**
	 * Action sets
     */
	std::vector<ActionSetFactory> action_sets_;
	std::vector<unsigned int> unconverged_action_sets_; // states where action sets can be still increased
	
public :
	GraphMVAS();
	virtual ~GraphMVAS() {}
};

/**
 * Mean-variance Pareto frontier generation using action sets
 */
class GraphMVPFAS : public GraphMVAS, public GraphMVPF<GraphStateWT<GraphStateVariance> >
{
private :
	/**
	 * Compute an initial policy whose mean value is the worst one
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
	GraphMVPFAS(const Problem& pb, double epsilon, double discount_factor); // throws
	
	/* Destructs this algorithm */
	virtual ~GraphMVPFAS() {}
};

/**
 * Mean-variance heuristic optimization using action sets
 */
class GraphMVHOAS : public GraphMVAS, public GraphMVHO<GraphStateWT<GraphStateVariance> >
{
private :
	/**
	 * Compute an initial policy whose mean value is the best one
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
	GraphMVHOAS(const Problem& pb, double minimum_mean, double maximum_variance, double epsilon, double discount_factor); // throws
	
	/* Destructs this algorithm */
	virtual ~GraphMVHOAS() {}
};

#endif // GRAPH_MVAS_H_
