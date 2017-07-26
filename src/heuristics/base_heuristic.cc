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

#include "base_heuristic.h"
#include "commons/base_exceptions.h"

// CLASS BaseHeuristic

BaseHeuristic::BaseHeuristic(double discount_factor)
try : discount_factor_(discount_factor)
{
	if ((discount_factor <= 0.0) || (discount_factor > 1.0))
		throw BaseException("discount factor must be in range ]0;1]", "BaseHeuristic::BaseHeuristic");
}
catch (...)
{
	throw;
}


void BaseHeuristic::set_discount_factor(double discount_factor)
{
	if ((discount_factor <= 0.0) || (discount_factor > 1.0))
		throw BaseException("discount factor must be in range ]0;1]", "BaseHeuristic::set_discount_factor");
	
	discount_factor_ = discount_factor;
}




// CLASS GraphHeuristic

template <class Tstate>
GraphHeuristic<Tstate>::GraphHeuristic(GraphMDP<Tstate>& mdp, double discount_factor)
try : BaseHeuristic(discount_factor), mdp_(mdp)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphHeuristic::GraphHeuristic");
	throw;
}


template <class Tstate>
double GraphHeuristic<Tstate>::value(const PddlState& st)
{
	Tstate& gst = mdp_.add_state(st);
	compute(gst);
	return gst.get_value();
}


// Explicit template instantiations
INSTANTIATE_STATE_TEMPLATE(GraphHeuristic)




// CLASS SymbolicHeuristic

SymbolicHeuristic::SymbolicHeuristic(SymbolicMDP& mdp, double discount_factor)
try : BaseHeuristic(discount_factor), mdp_(mdp)
{
}
catch (BaseException& error)
{
	error.push_function_backtrace("SymbolicHeuristic::SymbolicHeuristic");
	throw;
}


double SymbolicHeuristic::value(const PddlState& st)
{
	dd_node_ptr cudd_st = mdp_.add_state(st);
	compute(cudd_st);
	cudd_st = mdp_.get_state(st); // new atoms may have been added
	cudd_st = dd_node_ptr(Cudd_BddToAdd(dd_node_ptr::get_cudd_manager(), cudd_st.get()));
	dd_node_ptr valdd(Cudd_addApply(dd_node_ptr::get_cudd_manager(), Cudd_addTimes, mdp_.get_value_function().get(), cudd_st.get()));
	
	if (Cudd_IsConstant(valdd.get()) != 1)
		throw BaseException("state encoding error", "SymbolicHeuristic::value");
	
	return Cudd_V(valdd.get());
}
