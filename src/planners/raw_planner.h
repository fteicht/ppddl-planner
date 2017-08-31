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


#ifndef RAW_PLANNER_H
#define RAW_PLANNER_H

#include "base_planner.h"
#include "models/transitions_blackbox.h"
#include "commons/base_exceptions.h"

#include "config.h"
#ifdef HAVE_BOOST
#define BOOST_PARAMETER_MAX_ARITY 26
#include <boost/parameter.hpp>
#endif
#ifdef HAVE_BOOST_PYTHON
#define BOOST_PYTHON_MAX_ARITY 26
#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_VECTOR_SIZE 30 // can only be 20, 30, 40, ...
#include <boost/parameter/python.hpp>
#include <boost/python.hpp>
#endif

#ifdef HAVE_BOOST
BOOST_PARAMETER_NAME(domain)
BOOST_PARAMETER_NAME(problem)
BOOST_PARAMETER_NAME(algorithm)
BOOST_PARAMETER_NAME(heuristic)
BOOST_PARAMETER_NAME(encoding)
BOOST_PARAMETER_NAME(determinization)
BOOST_PARAMETER_NAME(deterministic_planner)
BOOST_PARAMETER_NAME(epsilon)
BOOST_PARAMETER_NAME(discount_factor)
BOOST_PARAMETER_NAME(time_threshold)
BOOST_PARAMETER_NAME(probabilistic_threshold)
BOOST_PARAMETER_NAME(nb_of_particles)
BOOST_PARAMETER_NAME(trajectories_max_depth)
BOOST_PARAMETER_NAME(goals_limit)
BOOST_PARAMETER_NAME(deterministic_probability_weight)
BOOST_PARAMETER_NAME(deterministic_reward_weight)
BOOST_PARAMETER_NAME(deterministic_planner_timeout)
BOOST_PARAMETER_NAME(deterministic_planner_max_trials)
BOOST_PARAMETER_NAME(proportion_updated_states)
BOOST_PARAMETER_NAME(minimum_mean)
BOOST_PARAMETER_NAME(maximum_variance)
BOOST_PARAMETER_NAME(compute_goal_statistics)
BOOST_PARAMETER_NAME(goal_statistics_algorithm)
BOOST_PARAMETER_NAME(warning_level)
BOOST_PARAMETER_NAME(verbosity_level)
#endif

class RawPlanner : public BasePlanner
{
private :
    const Domain* domain_;
	const Problem* problem_;
    
    long solving_time_;
    long resolving_time_;
    unsigned int nb_of_replannings_;
    bool compute_goal_statistics_;
    algorithm_t goal_statistics_algorithm_;
    
    void initialize(long time_threshold, int warn_level, int verb_level);
    
    /* Parses a domain file and a problem file */
    std::pair<const Domain*, const Problem*> parse_domain_and_problem(const std::string& domain, const std::string& problem);
    
    /* Checks if the policy need to be optimized in the given state */
	void check_solve(const PddlState& st);
    
    #ifdef HAVE_BOOST_PYTHON
    /** Get a PPDDL state from a python dictionary */
    void decode_state(const boost::python::dict& state, AtomSet& atoms, ValueMap& values);
    void encode_action(const Action& action, boost::python::dict& result);
    #endif

public :
	/* Constructs a planner. */
	RawPlanner(const Problem& problem, algorithm_t alg, heuristic_t heur, encoding_t enc,
			determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
			double epsilon = 0.000001, double discount_factor = 0.9, long time_threshold = 900000,
			double probabilistic_threshold = 0.1, unsigned int nb_of_particles = 10,
            unsigned int trajectories_max_depth = 100, unsigned int goals_limit = 100,
            double deterministic_probability_weight = 1.0, double deterministic_reward_weight = 10.0,
			unsigned int deterministic_planner_timeout = 5, unsigned int deterministic_planner_max_trials = 10,
			double proportion_updated_states = 0.01, double minimum_mean = 0.0, double maximum_variance = 0.0,
            bool compute_goal_statistics = false, algorithm_t goal_statistics_algorithm = ALGORITHM_GCE,
            int warn_level = 1, int verb_level = 0);
    
    RawPlanner(const std::string& domain, const std::string& problem, algorithm_t alg, heuristic_t heur, encoding_t enc,
			determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
			double epsilon = 0.000001, double discount_factor = 0.9, long time_threshold = 900000,
			double probabilistic_threshold = 0.1, unsigned int nb_of_particles = 10,
            unsigned int trajectories_max_depth = 100, unsigned int goals_limit = 100,
            double deterministic_probability_weight = 1.0, double deterministic_reward_weight = 10.0,
			unsigned int deterministic_planner_timeout = 5, unsigned int deterministic_planner_max_trials = 10,
			double proportion_updated_states = 0.01, double minimum_mean = 0.0, double maximum_variance = 0.0,
            bool compute_goal_statistics = false, algorithm_t goal_statistics_algorithm = ALGORITHM_GCE,
            int warn_level = 1, int verb_level = 0);
    
    #ifdef HAVE_BOOST
    template <class ArgumentPack>
    RawPlanner(const ArgumentPack& args)
    try : BasePlanner(*parse_domain_and_problem(args[_domain], args[_problem]).second,
                      args[_algorithm], args[_heuristic], args[_encoding],
                      args[_determinization], args[_deterministic_planner],
                      args[_epsilon | 0.000001], args[_discount_factor | 0.9],
                      args[_probabilistic_threshold | 0.1], args[_nb_of_particles | 10],
                      args[_trajectories_max_depth | 100], args[_goals_limit | 100],
                      args[_deterministic_probability_weight | 1.0], args[_deterministic_reward_weight | 10.0],
                      args[_deterministic_planner_timeout | 5], args[_deterministic_planner_max_trials | 10],
                      args[_proportion_updated_states | 0.01], args[_minimum_mean | 0.0], args[_maximum_variance | 0.0]),
                      solving_time_(0), resolving_time_(0), nb_of_replannings_(0),
                      compute_goal_statistics_(args[_compute_goal_statistics | false]),
                      goal_statistics_algorithm_(args[_goal_statistics_algorithm | ALGORITHM_GCE])
    {
        initialize(args[_time_threshold | 900000], args[_warning_level | 1], args[_verbosity_level | 0]);
    }
    catch(const BaseException& e)
    {
        std::cerr << "Unable to construct the planner! " << e.what() << std::endl;
    }
    #endif

	/* Deletes this planner. */
	virtual ~RawPlanner();

	/* Returns the optimal action in the given state */
	const Action& action(const PddlState& st);
    std::string action(const std::string& st); /* XML string as in the MDPsim convention */
    #ifdef HAVE_BOOST_PYTHON
    boost::python::str action(const boost::python::str& st); /* XML string as in the MDPsim convention */
    boost::python::dict action(const boost::python::dict& st);
    #endif

	/* Returns the optimal value in the given state */
	double value(const PddlState& st);
    double value(const std::string& st); /* XML string as in the MDPsim convention */
    #ifdef HAVE_BOOST_PYTHON
    double value(const boost::python::str& st); /* XML string as in the MDPsim convention */
    double value(const boost::python::dict& st);
    #endif
    
    /* Samples a next state when applying an action in a given state */
	//void next(const PddlState& income, const Action& action, PddlState& outcome, double& reward, bool& goal);
    //void next(const std::string& income, const std::string& action, std::string& outcome, double& reward, bool& goal); /* XML string as in the MDPsim convention */
    //#ifdef HAVE_BOOST_PYTHON
    //boost::python::tuple next(const boost::python::str& income, const boost::python::str& action); /* XML string as in the MDPsim convention */
    //boost::python::tuple value(const boost::python::dict& income, const boost::python::dict& action);
    //#endif
    
    /* Returns the (last) resolving time */
    inline long get_resolving_time() const {return resolving_time_;}

	/* Returns the solving time */
	inline long get_solving_time() const {return solving_time_;}
    
    /* Returns the number of replannings */
    inline unsigned int get_nb_of_replannings() const {return nb_of_replannings_;}
};


#ifdef HAVE_BOOST
class ParameterRawPlanner : public RawPlanner
{
public :
    BOOST_PARAMETER_CONSTRUCTOR(
        ParameterRawPlanner, (RawPlanner), tag,
        (required
            (domain, (std::string))
            (problem, (std::string))
            (algorithm, (algorithm_t))
            (heuristic, (heuristic_t))
            (encoding, (encoding_t))
            (determinization, (determinization_t))
            (deterministic_planner, (deterministic_planner_t))
        )
        (optional
            (epsilon, (double))
            (discount_factor, (double))
            (time_threshold, (long))
			(probabilistic_threshold, (double))
            (nb_of_particles, (unsigned int))
            (trajectories_max_depth, (unsigned int))
            (goals_limit, (unsigned int))
            (deterministic_probability_weight, (double))
            (deterministic_reward_weight, (double))
			(deterministic_planner_timeout, (unsigned int))
            (deterministic_planner_max_trials, (unsigned int))
			(proportion_updated_states, (double))
            (minimum_mean, (double))
            (maximum_variance, (double))
            (compute_goal_statistics, (bool))
            (goal_statistics_algorithm, (algorithm_t))
            (warning_level, (int))
            (verbosity_level, (int))
        )
    )
};
#endif

#endif
