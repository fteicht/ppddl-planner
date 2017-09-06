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

#ifndef HMAX_H_
#define HMAX_H_

#include <limits>
#include <ppddl_planner_mdpsim/problems.h>
#include "base_heuristic.h"

class HeuristicHmax : public virtual BaseHeuristic
{
protected :
	const Problem& problem_;
	std::vector<unsigned int> true_atoms_weights_;
	std::vector<unsigned int> false_atoms_weights_;
	
	unsigned int relaxed_hold(const StateFormula& formula, bool negate = false) const;
	bool relaxed_effect(const Effect& effect, unsigned int weight);
	
public :
	HeuristicHmax(double discount_factor, const Problem& pb); // throws
	virtual ~HeuristicHmax() {}
	
	virtual double value(const PddlState& st);
	
	inline const Problem& get_problem() const {return problem_;}
	bool is_true(const Atom& atom) const {return (true_atoms_weights_[atom.index()] != std::numeric_limits<unsigned int>::max());}
	bool is_false(const Atom& atom) const {return (false_atoms_weights_[atom.index()] != std::numeric_limits<unsigned int>::max());}
};


template <class Tstate>
class GraphHeuristicHmax : public GraphHeuristic<Tstate>, public HeuristicHmax
{
public :
	GraphHeuristicHmax(GraphMDP<Tstate>& mdp, double discount_factor, const Problem& pb);
	virtual ~GraphHeuristicHmax() {}
	
	virtual void compute(Tstate& st);
	virtual double value(const PddlState& st) {return HeuristicHmax::value(st);}
};


class SymbolicHeuristicHmax : public SymbolicHeuristic, public HeuristicHmax
{
public :
	SymbolicHeuristicHmax(SymbolicMDP& mdp, double discount_factor, const Problem& pb);
	virtual ~SymbolicHeuristicHmax() {}
	
	virtual void compute(const dd_node_ptr& states);
	virtual double value(const PddlState& st) {return HeuristicHmax::value(st);}
};

#endif /*HMAX_H_*/
