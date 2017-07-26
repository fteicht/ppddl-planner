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

#ifndef RELAXED_STEP_DISTANCE_H_
#define RELAXED_STEP_DISTANCE_H_

#include <mdpsim/problems.h>
#include "base_heuristic.h"
#include "config.h"

class RelaxedStepDistance : public virtual BaseHeuristic
{
protected :
	const Problem& problem_;
	AtomSet true_predicates_;
	AtomSet false_predicates_; // actually, its complement
#ifdef MEMORY_RELAXED_HEURISTICS
	std::map<std::pair<AtomSet, AtomSet>, double> heuristics_values_;
#endif
	
	bool update_predicates();
	bool goal_reached() const;
	
	bool relaxed_hold(const StateFormula& formula, bool negate = false) const;
	void relaxed_effect(const Effect& effect, AtomSet& true_prd, AtomSet& false_prd);
	
public :
	RelaxedStepDistance(double discount_factor, const Problem& pb); // throws
	virtual ~RelaxedStepDistance() {}
	
	virtual double value(const PddlState& st);
	
	inline const Problem& get_problem() const {return problem_;}
	bool is_true(const Atom& atom) const {return (true_predicates_.find(&atom) != true_predicates_.end());}
	bool is_false(const Atom& atom) const {return (false_predicates_.find(&atom) == false_predicates_.end());}
};


template <class Tstate>
class GraphRelaxedStepDistance : public GraphHeuristic<Tstate>, public RelaxedStepDistance
{
public :
	GraphRelaxedStepDistance(GraphMDP<Tstate>& mdp, double discount_factor, const Problem& pb);
	virtual ~GraphRelaxedStepDistance() {}
	
	virtual void compute(Tstate& st);
	virtual double value(const PddlState& st) {return RelaxedStepDistance::value(st);}
};


class SymbolicRelaxedStepDistance : public SymbolicHeuristic, public RelaxedStepDistance
{
public :
	SymbolicRelaxedStepDistance(SymbolicMDP& mdp, double discount_factor, const Problem& pb);
	virtual ~SymbolicRelaxedStepDistance() {}
	
	virtual void compute(const dd_node_ptr& states);
	virtual double value(const PddlState& st) {return RelaxedStepDistance::value(st);}
};

#endif /*RELAXED_STEP_DISTANCE_H_*/
