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

#ifndef MPO_DETERMINIZATION_H_
#define MPO_DETERMINIZATION_H_

#include <list>
#include "base_determinization.h"

class Effect;
class UpdateSet;

class MostProbableOutcomeDeterminizationGoal : public BaseDeterminization
{
private :
	/* Generates deterministic actions */
	virtual void generate_deterministic_actions();

	/* Generates the most probable effect of a given probabilistic effect */
	const Effect* generate_deterministic_effect(const Effect& probabilistic_effect) const;

	/* Renames the deterministic-made action so that it returns the original probabilistic action name */
	virtual std::string rename_action(const std::string& deterministic_name) const {return deterministic_name;}

public :
	/* Constructs a MPO determinization object */
	MostProbableOutcomeDeterminizationGoal(const Domain& dm, const Problem& pb) : BaseDeterminization(dm, pb) {}

	/* Deletes this MPO determinization object */
	virtual ~MostProbableOutcomeDeterminizationGoal() {}
};


class MostProbableOutcomeDeterminizationReward : public BaseDeterminizationWithFluents
{
private :
	/* Generates deterministic actions */
	virtual void generate_deterministic_actions();

	/* Generates the most probable effect of a given probabilistic effect */
	const Effect* generate_deterministic_effect(const Effect& probabilistic_effect) const;

	/* Renames the deterministic-made action so that it returns the original probabilistic action name */
	virtual std::string rename_action(const std::string& deterministic_name) const {return deterministic_name;}

public :
	/* Constructs a MPO determinization object */
	MostProbableOutcomeDeterminizationReward(const Domain& dm, const Problem& pb, unsigned int plan_length) : BaseDeterminizationWithFluents(dm, pb, plan_length) {}

	/* Deletes this MPO determinization object */
	virtual ~MostProbableOutcomeDeterminizationReward() {}
};

#endif /*MPO_DETERMINIZATION_H_*/
