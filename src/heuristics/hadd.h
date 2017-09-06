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

#ifndef HADD_H_
#define HADD_H_

#include <limits>
#include <ppddl_planner_mdpsim/problems.h>
#include "base_heuristic.h"

class HeuristicHadd : public virtual BaseHeuristic
{
protected :
	const Problem& problem_;
	std::vector<unsigned int> true_atoms_weights_;
	std::vector<unsigned int> false_atoms_weights_;
	
	unsigned int relaxed_hold(const StateFormula& formula, bool negate = false) const;
	bool relaxed_effect(const Effect& effect, unsigned int weight);
	
public :
	HeuristicHadd(double discount_factor, const Problem& pb); // throws
	virtual ~HeuristicHadd() {}
	
	virtual double value(const PddlState& st);
	
	inline const Problem& get_problem() const {return problem_;}
	bool is_true(const Atom& atom) const {return (true_atoms_weights_[atom.index()] != std::numeric_limits<unsigned int>::max());}
	bool is_false(const Atom& atom) const {return (false_atoms_weights_[atom.index()] != std::numeric_limits<unsigned int>::max());}
};


template <class Tstate>
class GraphHeuristicHadd : public GraphHeuristic<Tstate>, public HeuristicHadd
{
public :
	GraphHeuristicHadd(GraphMDP<Tstate>& mdp, double discount_factor, const Problem& pb);
	virtual ~GraphHeuristicHadd() {}
	
	virtual void compute(Tstate& st);
	virtual double value(const PddlState& st) {return HeuristicHadd::value(st);}
};


class SymbolicHeuristicHadd : public SymbolicHeuristic, public HeuristicHadd
{
public :
	SymbolicHeuristicHadd(SymbolicMDP& mdp, double discount_factor, const Problem& pb);
	virtual ~SymbolicHeuristicHadd() {}
	
	virtual void compute(const dd_node_ptr& states);
	virtual double value(const PddlState& st) {return HeuristicHadd::value(st);}
};

#endif /*HADD_H_*/
