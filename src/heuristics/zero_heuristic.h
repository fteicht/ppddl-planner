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

#ifndef ZERO_HEURISTIC_H_
#define ZERO_HEURISTIC_H_

#include "base_heuristic.h"

template <class Tstate>
class GraphZeroHeuristic : public GraphHeuristic<Tstate>
{
public :
	GraphZeroHeuristic(GraphMDP<Tstate>& mdp, double discount_factor)
	try : BaseHeuristic(discount_factor), GraphHeuristic<Tstate>(mdp, discount_factor)
	{
	}
	catch (BaseException& error)
	{
		error.push_function_backtrace("GraphZeroHeuristic::GraphZeroHeuristic");
		throw;
	}
	
	virtual ~GraphZeroHeuristic() {}
	virtual void compute(Tstate& st) {st.set_value(0.0);}
};


class SymbolicZeroHeuristic : public SymbolicHeuristic
{
public :
	SymbolicZeroHeuristic(SymbolicMDP& mdp, double discount_factor)
	try : BaseHeuristic(discount_factor), SymbolicHeuristic(mdp, discount_factor)
	{
	}
	catch (BaseException& error)
	{
		error.push_function_backtrace("SymbolicZeroHeuristic::SymbolicZeroHeuristic");
		throw;
	}
	
	virtual ~SymbolicZeroHeuristic() {}
	
	virtual void compute(const dd_node_ptr& states)
	{
		dd_node_ptr states_cmpl(Cudd_Not(states.get()));
		states_cmpl = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), states_cmpl.get()));
		mdp_._value_function_() = dd_node_ptr(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, mdp_.get_value_function().get(), states_cmpl.get()));
	}
};

#endif /*ZERO_HEURISTIC_H_*/
