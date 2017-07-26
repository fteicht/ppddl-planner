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

#ifndef BASE_HEURISTIC_H_
#define BASE_HEURISTIC_H_

#include "models/graph_mdp.h"
#include "models/symbolic_mdp.h"

class BaseHeuristic
{
protected :
	double discount_factor_;
	
public :
	BaseHeuristic(double discount_factor); // throws
	
	virtual ~BaseHeuristic() {}
	
	void set_discount_factor(double discount_factor); // throws	
	inline double get_discount_factor() const {return discount_factor_;}
	
	virtual double value(const PddlState& st) =0;
	virtual const BaseMDP* get_mdp() const =0; // Returns NULL if the heuristic does not depend on an MDP
};


template <class Tstate>
class GraphHeuristic : public virtual BaseHeuristic
{
public :
	typedef std::set<const Tstate*> states_set_t;
	
protected :
	GraphMDP<Tstate>& mdp_;
	
public :
	GraphHeuristic(GraphMDP<Tstate>& mdp, double discount_factor); // throws
	virtual ~GraphHeuristic() {}
	
	virtual void compute(Tstate& st) =0;
	virtual double value(const PddlState& st);
	
	virtual const BaseMDP* get_mdp() const {return &mdp_;}
};


class SymbolicHeuristic : public virtual BaseHeuristic
{
protected :
	SymbolicMDP& mdp_;
	
public :
	SymbolicHeuristic(SymbolicMDP& mdp, double discount_factor); // throws
	virtual ~SymbolicHeuristic() {}
	
	virtual void compute(const dd_node_ptr& states) =0;
	virtual double value(const PddlState& st);
	
	virtual const BaseMDP* get_mdp() const {return &mdp_;}
};

#endif /*BASE_HEURISTIC_H_*/
