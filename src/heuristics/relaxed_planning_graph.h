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

#ifndef RELAXED_PLANNING_GRAPH_H_
#define RELAXED_PLANNING_GRAPH_H_

#include <ppddl_planner_mdpsim/problems.h>
#include "base_heuristic.h"
#include "models/relaxed_mdp_graph.h"

class RelaxedPlanningGraph : public virtual BaseHeuristic
{
	private:
		const Problem& problem_;
		RelaxedMDPGraph* mdp_pg;

		bool reached(const AtomSet& current, AtomSet& previous);

	public :
		RelaxedPlanningGraph(double discount_factor, const Problem& pb);
		~RelaxedPlanningGraph();

		virtual double value(const PddlState& st);
};

template <class Tstate>
class GraphRelaxedPlanningGraph : public GraphHeuristic<Tstate>, public RelaxedPlanningGraph
{
public :
	GraphRelaxedPlanningGraph(GraphMDP<Tstate>& mdp, double discount_factor, const Problem& pb);
	virtual ~GraphRelaxedPlanningGraph() {}

	virtual void compute(Tstate& st);
	virtual double value(const PddlState& st) {return RelaxedPlanningGraph::value(st);}
};


class SymbolicRelaxedPlanningGraph : public SymbolicHeuristic, public RelaxedPlanningGraph
{
public :
	SymbolicRelaxedPlanningGraph(SymbolicMDP& mdp, double discount_factor, const Problem& pb);
	virtual ~SymbolicRelaxedPlanningGraph() {}

	virtual void compute(const dd_node_ptr& states);
	virtual double value(const PddlState& st) {return RelaxedPlanningGraph::value(st);}
};

#endif /*RELAXED_PLANNING_GRAPH_H_*/
