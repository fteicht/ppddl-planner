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

#ifndef RDDL_PLANNER_H_
#define RDDL_PLANNER_H_

#include <cmath>
#include <cstdlib>

#include "base_planner.h"

class RddlPlanner : public BasePlanner, public Planner
{
protected :
	/* time of the round beginning */
	long round_time_;

	/* return a random action if no more time is available */
	const Action* randomAction(const AtomSet& atoms, const ValueMap& values);

public :
	/* Constructs a planner. */
	RddlPlanner(const Problem& problem, algorithm_t alg, heuristic_t heur, encoding_t enc,
			determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
			double epsilon = 0.000001, double discount_factor = 0.9, double probabilistic_threshold = 0.1,
			unsigned int nb_of_particles = 10, unsigned int trajectories_max_depth = 100, unsigned int goals_limit = 100,
			double deterministic_probability_weight = 1.0, double deterministic_reward_weight = 10.0,
			unsigned int deterministic_planner_timeout = 5, unsigned int deterministic_planner_max_trials = 10,
			double proportion_updated_states = 0.01, double minimum_mean = 0.0, double maximum_variance = 0.0);

	/* Deletes this planner. */
	virtual ~RddlPlanner() {}

	/* Called to initialize a round. */
        virtual	void initRound(  int numberOfRounds,  long remainingTime);

	/* Called to return an action for the given state. */
	virtual const Action* decideAction(const AtomSet& atoms, const ValueMap& values, long time);

	/* Called to finalize a round. */
        virtual void endRound( void ) { }
	virtual void endRound(double const& r);

        /* For compatibility with mdpsim */
	virtual void initRound();
	virtual const Action* decideAction(const AtomSet& atoms, const ValueMap& values);
};

#endif /*RDDL_PLANNER_H_*/
