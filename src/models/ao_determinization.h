/*
   ppddl-planner - client for IPPC'08

   Copyright (C) 2009 Florent Teichteil-Koenigsbuch and Guillaume Infantes and Ugur Kuter

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

#ifndef AO_DETERMINIZATION_H_
#define AO_DETERMINIZATION_H_

#include <list>
#include "base_determinization.h"

class Effect;
class UpdateSet;

class AllOutcomesDeterminizationGoal : public BaseDeterminization
{
private :
	/* Generates deterministic actions */
	virtual void generate_deterministic_actions();

	/* Generates deterministic effects for a given probabilistic effect */
	void generate_deterministic_effects(const Effect& probabilistic_effect, EffectList& deterministic_effects) const;

	/* Renames the deterministic-made action so that it returns the original probabilistic action name */
	virtual std::string rename_action(const std::string& deterministic_name) const;

public :
	/* Constructs a AO determinization object */
	AllOutcomesDeterminizationGoal(const Domain& dm, const Problem& pb) : BaseDeterminization(dm, pb) {}

	/* Deletes this AO determinization object */
	virtual ~AllOutcomesDeterminizationGoal() {}
};

class AllOutcomesDeterminizationReward : public BaseDeterminizationWithFluents
{
private :
	/* Generates deterministic actions */
	virtual void generate_deterministic_actions();

	/* Generates deterministic effects for a given probabilistic effect */
	void generate_deterministic_effects(const Effect& probabilistic_effect, EffectList& deterministic_effects) const;

	/* Renames the deterministic-made action so that it returns the original probabilistic action name */
	virtual std::string rename_action(const std::string& deterministic_name) const;

public :
	/* Constructs a AO determinization object */
	AllOutcomesDeterminizationReward(const Domain& dm, const Problem& pb, unsigned int plan_length) : BaseDeterminizationWithFluents(dm, pb, plan_length) {}

	/* Deletes this AO determinization object */
	virtual ~AllOutcomesDeterminizationReward() {}
};


#endif /*AO_DETERMINIZATION_H_*/
