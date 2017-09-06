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

#ifndef BASE_PLANNER_H_
#define BASE_PLANNER_H_

#include <ppddl_planner_mdpsim/client.h>


typedef enum
{
	ALGORITHM_VI,
	ALGORITHM_FSP,
	ALGORITHM_FSP_STAR,
	ALGORITHM_TRFSP_STAR,
	ALGORITHM_BFDT,
	ALGORITHM_PSP_STAR,
	ALGORITHM_FDP,
	ALGORITHM_LAO,
	ALGORITHM_IMPROVED_LAO,
	ALGORITHM_RTDP,
	ALGORITHM_LRTDP,
	ALGORITHM_LDFS,
	ALGORITHM_OD,
	ALGORITHM_RDPG,
	ALGORITHM_RDRG,
	ALGORITHM_RDBG,
	ALGORITHM_RDRD,
	ALGORITHM_RDRO,
	ALGORITHM_RDRD_STAR,
	ALGORITHM_RDRO_STAR,
    ALGORITHM_GCO,
    ALGORITHM_GCE,
    ALGORITHM_MVPFPI,
	ALGORITHM_MVHOPI,
	ALGORITHM_MVPFAS,
	ALGORITHM_MVHOAS,
	ALGORITHM_MVPFSSP
}
algorithm_t;


typedef enum
{
	HEURISTIC_ZERO,
	HEURISTIC_PDDL_STEP_DISTANCE,
	HEURISTIC_NATIVE_STEP_DISTANCE,
	HEURISTIC_RELAXED_STEP_DISTANCE,
	HEURISTIC_RELAXED_REWARDS,
	HEURISTIC_RELAXED_PLANNING_GRAPH,
	HEURISTIC_BOUNDS,
	HEURISTIC_HADD,
	HEURISTIC_HMAX,
	HEURISTIC_HDET
}
heuristic_t;


typedef enum
{
	ENCODING_GRAPH,
	ENCODING_SYMBOLIC
}
encoding_t;


typedef enum
{
	DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL,
	DETERMINIZATION_MOST_PROBABLE_OUTCOME_REWARD,
	DETERMINIZATION_ALL_OUTCOMES_GOAL,
	DETERMINIZATION_ALL_OUTCOMES_REWARD
}
determinization_t;


typedef enum
{
	DETERMINISTIC_PLANNER_FF,
	DETERMINISTIC_PLANNER_MFF
}
deterministic_planner_t;


class BaseAlgorithm;

class BasePlanner
{
protected :
	const Problem& problem_;
	BaseAlgorithm* algorithm_;
	long time_threshold_;

	algorithm_t algorithm_type_;
	heuristic_t heuristic_type_;
	encoding_t encoding_type_;
	determinization_t determinization_type_;
	deterministic_planner_t deterministic_planner_type_;
	bool random_algorithm_;

	double epsilon_;
	double discount_factor_;

	double probabilistic_threshold_;
	unsigned int nb_of_particles_;
	unsigned int trajectories_max_depth_;
	unsigned int goals_limit_;

	double deterministic_probability_weight_;
	double deterministic_reward_weight_;
	unsigned int deterministic_planner_timeout_;
	unsigned int deterministic_planner_max_trials_;

	double proportion_updated_states_;
	double minimum_mean_;
	double maximum_variance_;

	/* Returns the current time in milliseconds. */
	static long get_time_milli();

	/* Returns TRUE if RAM is available */
	static bool ram_available();

	/* Constructs the algorithm */
	void construct_algorithm();

public :
	/* Constructs a planner. */
	BasePlanner(const Problem& problem, algorithm_t alg, heuristic_t heur, encoding_t enc,
			determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
			double epsilon = 0.000001, double discount_factor = 0.9, double probabilistic_threshold = 0.1,
			unsigned int nb_of_particles = 10, unsigned int trajectories_max_depth = 100, unsigned int goals_limit = 100,
			double deterministic_probability_weight = 1.0, double deterministic_reward_weight = 10.0,
			unsigned int deterministic_planner_timeout = 5, unsigned int deterministic_planner_max_trials = 10,
			double proportion_updated_states = 0.01, double minimum_mean = 0.0, double maximum_variance = 0.0);

	/* Deletes this planner. */
	virtual ~BasePlanner();

	/* Returns the problem solved */
	inline const Problem& get_problem() const {return problem_;}

	/* Returns the algorithm */
	inline const BaseAlgorithm& get_algorithm() const {return *algorithm_;}

	/* Returns the algorithm type */
	inline algorithm_t get_algorithm_type() const {return algorithm_type_;}

	/* Returns the heuristic type */
	inline heuristic_t get_heuristic_type() const {return heuristic_type_;}

	/* Returns the encoding type */
	inline encoding_t get_encoding_type() const {return encoding_type_;}

	/* Returns the determinization type */
	inline determinization_t get_determinization_type() const {return determinization_type_;}

	/* Returns the deterministic planner type */
	inline deterministic_planner_t get_deterministic_planner_type() const {return deterministic_planner_type_;}

	/* Returns the probabilistic threshold */
	inline double get_probabilistic_threshold() const {return probabilistic_threshold_;}

	/* Returns the number of particles */
	inline unsigned int get_nb_of_particles() const {return nb_of_particles_;}

	/* Returns the trajectories maximum depth */
	inline unsigned int get_trajectories_max_depth() const {return trajectories_max_depth_;}

	/* Returns the limit of subgoals number (for subgoals generation algorithms) */
	inline unsigned int get_goals_limit() const {return goals_limit_;}

    /* Returns the weight of the (log) probability fluent in the deterministic planner's goal formula (for reward-based MDPs) */
	inline double get_deterministic_probability_weight() const {return deterministic_probability_weight_;}

    /* Returns the weight of the reward fluent in the deterministic planner's goal formula (for reward-based MDPs) */
	inline double get_deterministic_reward_weight() const {return deterministic_reward_weight_;}

	/* Returns the timeout (in seconds) of the deterministic planner's engine */
	inline unsigned int get_deterministic_planner_timeout() const {return deterministic_planner_timeout_;}

	/* Returns the maximum number of trials for the deterministic planner to find a solution (for reward-based MDPs) */
	inline unsigned int get_deterministic_planner_max_trials() const {return deterministic_planner_max_trials_;}

	/* Returns true if the algorithm is random */
	inline bool random_algorithm() const {return random_algorithm_;}

	/* Returns the proportion of updated states */
	inline double proportion_updated_states() const {return proportion_updated_states_;}

	/* Returns the minimum allowed mean */
	inline double minimum_mean() const {return minimum_mean_;}

	/* Returns the maximum allowed variance */
	inline double maximum_variance() const {return maximum_variance_;}
};

#endif /*BASE_PLANNER_H_*/
