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

#ifndef FF_PLANNER_H_
#define FF_PLANNER_H_

#include "deterministic_planner.h"

class FFPlanner : public DeterministicPlanner
{
protected :
	static const unsigned int stdout_line_length_ = 1000;
	static const unsigned int action_nb_string_length_ = 10;

	virtual void read_plan(FILE* ff_stdout, plan_t& plan);
	void push_action(plan_t& plan, const char* action_str);

public :
	/* Constructs a FF planner */
	FFPlanner(const BaseDeterminization& det) : DeterministicPlanner(det) {}

	/* Deletes this FF planner */
	virtual ~FFPlanner() {}
};


class SFFPlanner : public FFPlanner
{
private :
    virtual void execute_planner(bool optimization_mode = false) const;
    virtual std::string planner_command(bool optimization_mode = false) const;

public :
    /* Constructs a symbolic (regular) FF planner */
	SFFPlanner(const BaseDeterminization& det) : FFPlanner(det) {}

	/* Deletes this symbolic (regular) FF planner */
	virtual ~SFFPlanner() {}
};


class NFFPlanner : public FFPlanner
{
private :
    virtual void execute_planner(bool optimization_mode = false) const;
    virtual std::string planner_command(bool optimization_mode = false) const;

public :
    /* Constructs a numeric (metric) FF planner */
	NFFPlanner(const BaseDeterminization& det) : FFPlanner(det) {}

	/* Deletes this numeric (metric) FF planner */
	virtual ~NFFPlanner() {}
};

#endif /*FF_PLANNER_H_*/
