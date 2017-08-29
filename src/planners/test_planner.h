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

#ifndef TEST_PLANNER_H_
#define TEST_PLANNER_H_

#include <mdpsim/states.h>
#include "base_planner.h"
#include "models/transitions_blackbox.h"

typedef enum
{
	TEST_PLANNER_COMPLETE,
	TEST_PLANNER_ANYTIME
}
test_planner_t;

class SimulationTestPlanner : public BasePlanner
{
public :
	struct SimulationInfo
	{
		double average_cumulated_value_;
		double goal_reached_percentage_;
		double average_turns_number_;
		double average_time_per_trial_;
		unsigned int nb_of_replannings_;
		double average_planning_time_;
		double time_out_percentage_;
	};

protected :
	const Problem& problem_;
	const State* current_state_;
	long initial_time_;
	long solving_time_;
	long resolving_time_;
	unsigned int trials_number_;
	unsigned int trials_max_depth_;
	unsigned int nb_of_replannings_;
    bool compute_goal_statistics_;
    algorithm_t goal_statistics_algorithm_;

	/* Checks if the policy need to be optimized in the given state */
	virtual void check_solve(const PddlState& st) =0;

public :
	/* Constructs a planner. */
	SimulationTestPlanner(const Problem& problem, algorithm_t alg, heuristic_t heur, encoding_t enc,
			determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
			double epsilon = 0.000001, double discount_factor = 0.9, long time_threshold = 900000,
			unsigned int trials_number = 100, unsigned int trials_max_depth = 1000,
			double probabilistic_threshold = 0.1, unsigned int nb_of_particles = 10,
            unsigned int trajectories_max_depth = 100, unsigned int goals_limit = 100,
            double deterministic_probability_weight = 1.0, double deterministic_reward_weight = 10.0,
			unsigned int deterministic_planner_timeout = 5, unsigned int deterministic_planner_max_trials = 10,
			double proportion_updated_states = 0.01, double minimum_mean = 0.0, double maximum_variance = 0.0,
            bool compute_goal_statistics = false, algorithm_t goal_statistics_algorithm = ALGORITHM_GCE);

	/* Deletes this planner. */
	virtual ~SimulationTestPlanner();

	/* Launches the tests */
	SimulationInfo launch();

	/* Returns the optimal action in the given state */
	const Action& action(const State& st);

	/* Returns the optimal value in the given state */
	double value(const State& st);

	/* Returns the solving time */
	inline long get_solving_time() const {return solving_time_;}
};


class SimulationTestCompletePlanner : public SimulationTestPlanner
{
private :
	/* Checks if the policy need to be optimized in the given state */
	virtual void check_solve(const PddlState& st);

public :
	public :
	/* Constructs a planner. */
	SimulationTestCompletePlanner(const Problem& problem, algorithm_t alg, heuristic_t heur, encoding_t enc,
			determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
			double epsilon = 0.000001, double discount_factor = 0.9, long time_threshold = 900000,
			unsigned int trials_number = 100, unsigned int trials_max_depth = 1000,
			double probabilistic_threshold = 0.1, unsigned int nb_of_particles = 10,
            unsigned int trajectories_max_depth = 100, unsigned int goals_limit = 100,
            double deterministic_probability_weight = 1.0, double deterministic_reward_weight = 10.0,
			unsigned int deterministic_planner_timeout = 5, unsigned int deterministic_planner_max_trials = 10,
			double proportion_updated_states = 0.01, double minimum_mean = 0.0, double maximum_variance = 0.0,
            bool compute_goal_statistics = false, algorithm_t goal_statistics_algorithm = ALGORITHM_GCE);

	/* Deletes this planner. */
	virtual ~SimulationTestCompletePlanner() {}
};


class SimulationTestAnytimePlanner : public SimulationTestPlanner
{
private :
	/* Checks if the policy need to be optimized in the given state */
	virtual void check_solve(const PddlState& st);

public :
	public :
	/* Constructs a planner. */
	SimulationTestAnytimePlanner(const Problem& problem, algorithm_t alg, heuristic_t heur, encoding_t enc,
			determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
			double epsilon = 0.000001, double discount_factor = 0.9, long time_threshold = 900000,
			unsigned int trials_number = 100, unsigned int trials_max_depth = 1000,
			double probabilistic_threshold = 0.1, unsigned int nb_of_particles = 10,
            unsigned int trajectories_max_depth = 100, unsigned int goals_limit = 100,
            double deterministic_probability_weight = 1.0, double deterministic_reward_weight = 10.0,
			unsigned int deterministic_planner_timeout = 5, unsigned int deterministic_planner_max_trials = 10,
			double proportion_updated_states = 0.01, double minimum_mean = 0.0, double maximum_variance = 0.0,
            bool compute_goal_statistics = false, algorithm_t goal_statistics_algorithm = ALGORITHM_GCE);

	/* Deletes this planner. */
	virtual ~SimulationTestAnytimePlanner() {}
};

#endif /*TEST_PLANNER_H_*/
