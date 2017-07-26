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

#ifndef DETERMINIZATION_HEURISTIC_H_
#define DETERMINIZATION_HEURISTIC_H_

#include "base_heuristic.h"
#include "planners/base_planner.h"

class BaseDeterminization;
class DeterministicPlanner;

class DeterminizationHeuristic : public virtual BaseHeuristic
{
protected :
	BaseDeterminization* determinization_;
	DeterministicPlanner* deterministic_planner_;

	determinization_t determinization_type_;
	deterministic_planner_t deterministic_planner_type_;

	const Problem& problem_;

public :
	/* Constructs a determinization heuristic */
	DeterminizationHeuristic(const Problem& pb, double discount_factor, unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type); // throws

	/* Deletes this determinization heuristic */
	virtual ~DeterminizationHeuristic();

	/* Returns the determinization type */
	inline determinization_t get_determinization_type() const {return determinization_type_;}

	/* Returns the deterministic planner type */
	inline deterministic_planner_t get_deterministic_planner_type() const {return deterministic_planner_type_;}

	/* Returns the determinization object */
	inline const BaseDeterminization& get_determinization() const {return (*determinization_);}

	/* Returns the deterministic planner object */
	inline const DeterministicPlanner& get_deterministic_planner() const {return (*deterministic_planner_);}
};

#endif /*DETERMINIZATION_HEURISTIC_H_*/
