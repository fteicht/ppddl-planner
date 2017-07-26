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

#ifndef BASE_MDP_H_
#define BASE_MDP_H_

#include <string>
#include "transitions_blackbox.h"

class BaseMDP : public TransitionsBlackboxUser
{
public :
	BaseMDP(const TransitionsBlackbox& tb) : TransitionsBlackboxUser(tb) {}
	virtual ~BaseMDP() {}
	
	/* Returns the policy of the given state */
	virtual const Action& get_policy(const PddlState& st) const =0; // throws
	
	/* Returns the value of the given state */
	virtual double get_value(const PddlState& st) const =0; // throws
	
	/* Returns true if the given state has been explored */
	virtual bool explored(const PddlState& st) const =0;
	
	/* Returns true if the given state has been expanded */
	virtual bool expanded(const PddlState& st) const =0;
	
	/* Returns true if the given state is a goal (among explored states) */
	virtual bool goal(const PddlState& st) const =0;
	
	/* Prints the MDP model */
	virtual std::string print_model() const =0;
	
	/* Prints the MDP solution */
	virtual std::string print_solution() const =0;
};

#endif /*BASE_MDP_H_*/
