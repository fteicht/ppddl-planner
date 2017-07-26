/*
   ppddl-planner - client for IPPC'08

   Copyright (C) 2011 Florent Teichteil-Koenigsbuch and Guillaume Infantes and Ugur Kuter

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

#ifndef HDET_H_
#define HDET_H_

#include "determinization_heuristic.h"

class HeuristicHdet : public DeterminizationHeuristic
{
public :
	/* Constructs a Hdet heuristic */
	HeuristicHdet(const Problem& pb, double discount_factor, unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type); // throws

	/* Deletes this Hdet heuristic */
	virtual ~HeuristicHdet() {}

	virtual double value(const PddlState& st);
};


template <class Tstate>
class GraphHeuristicHdet : public GraphHeuristic<Tstate>, public HeuristicHdet
{
public :
	GraphHeuristicHdet(GraphMDP<Tstate>& mdp, const Problem& pb, double discount_factor, unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type);
	virtual ~GraphHeuristicHdet() {}

	virtual void compute(Tstate& st);
	virtual double value(const PddlState& st) {return HeuristicHdet::value(st);}
};


class SymbolicHeuristicHdet : public SymbolicHeuristic, public HeuristicHdet
{
public :
	SymbolicHeuristicHdet(SymbolicMDP& mdp, const Problem& pb, double discount_factor, unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type);
	virtual ~SymbolicHeuristicHdet() {}

	virtual void compute(const dd_node_ptr& states);
	virtual double value(const PddlState& st) {return HeuristicHdet::value(st);}
};

#endif /*HDET_H_*/
