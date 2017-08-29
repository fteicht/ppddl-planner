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
#ifdef HAVE_BOOST_PYTHON
#define BOOST_PYTHON_MAX_ARITY 26
#include <boost/python.hpp>
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
    
    /* Returns the (last) resolving time */
    inline long get_resolving_time() const {return resolving_time_;}

	/* Returns the solving time */
	inline long get_solving_time() const {return solving_time_;}
    
    /* Returns the number of replannings */
    inline unsigned int get_nb_of_replannings() const {return nb_of_replannings_;}
};


#ifdef HAVE_BOOST_PYTHON
boost::python::str (RawPlanner::*action_str)(const boost::python::str&) = &RawPlanner::action;
boost::python::dict (RawPlanner::*action_dict)(const boost::python::dict&) = &RawPlanner::action;
double (RawPlanner::*value_str)(const boost::python::str&) = &RawPlanner::value;
double (RawPlanner::*value_dict)(const boost::python::dict&) = &RawPlanner::value;
    
BOOST_PYTHON_MODULE(libppddl_planner)
{
    boost::python::enum_<algorithm_t>("algorithm")
                                          .value("vi", ALGORITHM_VI)
                                          .value("fsp", ALGORITHM_FSP)
                                          .value("fsp-star", ALGORITHM_FSP_STAR)
                                          .value("trfsp_star", ALGORITHM_TRFSP_STAR)
                                          .value("bfdt", ALGORITHM_BFDT)
                                          .value("psp_star", ALGORITHM_PSP_STAR)
                                          .value("fdp", ALGORITHM_FDP)
                                          .value("lao", ALGORITHM_LAO)
                                          .value("ilao", ALGORITHM_IMPROVED_LAO)
                                          .value("rtdp", ALGORITHM_RTDP)
                                          .value("lrtdp", ALGORITHM_LRTDP)
                                          .value("ldfs", ALGORITHM_LDFS)
                                          .value("od", ALGORITHM_OD)
                                          .value("rdpg", ALGORITHM_RDPG)
                                          .value("rdrg", ALGORITHM_RDRG)
                                          .value("rdbg", ALGORITHM_RDBG)
                                          .value("rdrd", ALGORITHM_RDRD)
                                          .value("rdro", ALGORITHM_RDRO)
                                          .value("rdrd_star", ALGORITHM_RDRD_STAR)
                                          .value("rdro_star", ALGORITHM_RDRO_STAR)
                                          .value("gco", ALGORITHM_GCO)
                                          .value("gce", ALGORITHM_GCE)
                                          .value("mvpfpi", ALGORITHM_MVPFPI)
                                          .value("mvhopi", ALGORITHM_MVHOPI)
                                          .value("mvpfas", ALGORITHM_MVPFAS)
                                          .value("mvhoas", ALGORITHM_MVHOAS)
                                          .value("mvpfssp", ALGORITHM_MVPFSSP);
    boost::python::enum_<heuristic_t>("heuristic")
                                          .value("zero", HEURISTIC_ZERO)
                                          .value("pddl_step_distance", HEURISTIC_PDDL_STEP_DISTANCE)
                                          .value("native_step_distance", HEURISTIC_NATIVE_STEP_DISTANCE)
                                          .value("relaxed_step_distance", HEURISTIC_RELAXED_STEP_DISTANCE)
                                          .value("relaxed_rewards", HEURISTIC_RELAXED_REWARDS)
                                          .value("relaxed_planning_graph", HEURISTIC_RELAXED_PLANNING_GRAPH)
                                          .value("bounds", HEURISTIC_BOUNDS)
                                          .value("hadd", HEURISTIC_HADD)
                                          .value("hmax", HEURISTIC_HMAX)
                                          .value("hdet", HEURISTIC_HDET);
    boost::python::enum_<encoding_t>("encoding")
                                         .value("graph", ENCODING_GRAPH)
                                         .value("symbolic", ENCODING_SYMBOLIC);
    boost::python::enum_<determinization_t>("determinization")
                                                .value("most_probable_outcome_goal", DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL)
                                                .value("most_probable_outcome_reward", DETERMINIZATION_MOST_PROBABLE_OUTCOME_REWARD)
                                                .value("all_outcomes_goal", DETERMINIZATION_ALL_OUTCOMES_GOAL)
                                                .value("all_outcomes_reward", DETERMINIZATION_ALL_OUTCOMES_REWARD);
    boost::python::enum_<deterministic_planner_t>("deterministic_planner")
                                                      .value("ff", DETERMINISTIC_PLANNER_FF)
                                                      .value("mff", DETERMINISTIC_PLANNER_MFF);
    boost::python::class_<RawPlanner>("Planner",
                                          boost::python::init<const std::string&, const std::string&,
                                                              algorithm_t, heuristic_t, encoding_t,
			                                                  determinization_t, deterministic_planner_t,
			                                                  boost::python::optional<double, double, long,
			                                                  double, unsigned int,
                                                              unsigned int, unsigned int,
                                                              double, double,
                                                              unsigned int, unsigned int,
			                                                  double, double, double,
                                                              bool, algorithm_t,
                                                              int, int> >((
                                                                 boost::python::arg("domain"),
                                                                 boost::python::arg("problem"),
                                                                 boost::python::arg("algorithm"),
                                                                 boost::python::arg("heuristic"),
                                                                 boost::python::arg("encoding"),
                                                                 boost::python::arg("determinization"),
                                                                 boost::python::arg("deterministic_planner"),
                                                                 boost::python::arg("epsilon"),
                                                                 boost::python::arg("discount_factor"),
                                                                 boost::python::arg("time_threshold"),
                                                                 boost::python::arg("probabilistic_threshold"),
                                                                 boost::python::arg("nb_of_particles"),
                                                                 boost::python::arg("trajectories_max_depth"),
                                                                 boost::python::arg("goals_limit"),
                                                                 boost::python::arg("deterministic_probability_weight"),
                                                                 boost::python::arg("deterministic_reward_weight"),
                                                                 boost::python::arg("deterministic_planner_timeout"),
                                                                 boost::python::arg("deterministic_planner_max_trials"),
                                                                 boost::python::arg("proportion_updated_states"),
                                                                 boost::python::arg("minimum_mean"),
                                                                 boost::python::arg("maximum_variance"),
                                                                 boost::python::arg("compute_goal_statistics"),
                                                                 boost::python::arg("goal_statistics_algorithm"),
                                                                 boost::python::arg("warning_level"),
                                                                 boost::python::arg("verbosity_level"))))
                                          .def("action", action_str, (boost::python::arg("state")))
                                          .def("action", action_dict, (boost::python::arg("state")))
                                          .def("value", value_str, (boost::python::arg("state")))
                                          .def("value", value_dict, (boost::python::arg("state")))
                                          .def("get_resolving_time", &RawPlanner::get_resolving_time)
                                          .def("get_solving_time", &RawPlanner::get_solving_time)
                                          .def("get_nb_of_replannings", &RawPlanner::get_nb_of_replannings)
    ;
}
#endif

#endif
