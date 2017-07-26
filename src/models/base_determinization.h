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

#ifndef BASE_DETERMINIZATION_H_
#define BASE_DETERMINIZATION_H_

#include <string>
#include <list>

class Domain;
class Problem;
class PddlState;
class TransitionsBlackbox;

class BaseDeterminization
{
protected :
	const Domain& domain_;
	const Problem& problem_;
	std::string domain_file_name_;
	std::string problem_file_name_;
	ActionSchemaMap deterministic_action_schemas_;
	ActionSet deterministic_actions_;
	static const unsigned int dirpath_length_ = 100;

	/* Generates deterministic actions */
	virtual void generate_deterministic_actions() =0;

	/* Adds a deterministic action schema and generates its instantiations */
	void add_deterministic_action_schema(const ActionSchema& probabilistic_action, const std::string& deterministic_name, const Effect& deterministic_effect);

	/* Writes requirements */
	void write_requirements(std::ofstream& cdf) const;

	/* Writes types */
	void write_types(std::ofstream& cdf) const;
	void collect_types(std::map<std::string, std::set<std::string> >& types_subtypes_map, std::set<std::string>& top_types) const;
	void _write_types_(std::ofstream& cdf, const std::string& top_type, const std::set<std::string>& sub_types, const std::map<std::string, std::set<std::string> >& types_subtypes_map) const;

	/* Writes constants */
	void write_constants(std::ofstream& cdf) const;

	/* Writes predicates */
	void write_predicates(std::ofstream& cdf) const;

	/* Writes functions */
	void write_functions(std::ofstream& cdf) const;

	/* Writes actions */
	void write_actions(std::ofstream& cdf) const;

	/* Writes objects */
	void write_objects(std::ofstream& cdf) const;

	/* Writes an init formula */
	void write_init_formula(std::ofstream& cdf, const AtomSet& init_atoms, const ValueMap& init_values) const;

	/* Writes a goal formula */
	void write_goal_formula(std::ofstream& cdf, const StateFormula& goal_states) const;

	/* Writes an optimization (maximization) formula */
	void write_optimization_formula(std::ofstream& cdf, const Expression& optimization_expression) const;

	/* Finds the probabilistic action corresponding to the given deterministic action */
	const Action& deterministic_to_probabilistic_action(const Action& deterministic_action) const;

	/* Renames the deterministic-made action so that it returns the original probabilistic action name */
	virtual std::string rename_action(const std::string& deterministic_name) const =0;

	/* Adds artificial fluent values like reward, planner-probability, planner-step-counter ... */
	virtual void add_artificial_values(const TransitionsBlackbox& tb, PddlState& outcome) const;

	/* Removes artificial fluent values like reward, planner-probability, planner-step-counter ... */
	virtual void remove_artificial_values(const TransitionsBlackbox& tb, PddlState& outcome) const;

public :
	/* Constructs a determinization object */
	BaseDeterminization(const Domain& dm, const Problem& pb);

	/* Deletes this determinization object */
	virtual ~BaseDeterminization();

	/* Gets the probabilistic domain */
	inline const Domain& get_domain() const {return domain_;}

	/* Gets the probabilistic problem */
	inline const Problem& get_problem() const {return problem_;}

	/* Makes the domain deterministic */
	void make_domain();

	/* Makes a problem */
	virtual void make_problem(const AtomSet& init_atoms, const ValueMap& init_values, const StateFormula& goal_states, const Expression* optimization_expression = NULL);

	/* Gets the deterministic domain file name */
	inline const std::string& get_domain_file_name() const {return domain_file_name_;}

	/* Gets the name of the last generated problem file */
	inline const std::string& get_problem_file_name() const {return problem_file_name_;}

	/* Returns the probabilistic action corresponding to the given action tokens (action's name, then list of action's parameters) */
	const Action* action(const std::list<std::string>& action_tokens) const;

	/* Returns the outcome (must be manually deleted) of the given action */
	void effect(const TransitionsBlackbox& tb, const PddlState& income, const Action& deterministic_action, PddlState& outcome, const Action*& probabilistic_action) const;
};


class BaseDeterminizationWithFluents : public BaseDeterminization
{
protected :
    /* Deterministic planner's plan length (command line parameter: trajectories-max-depth) */
    unsigned int plan_length_;

    /* Probability fluent */
	const Fluent* probability_fluent_;

	/* Reward fluent */
	const Fluent* reward_fluent_;

	/* Step counter to force the deterministic planner to find a plan of a given length (command line parameter: trajectories-max-depth) */
	const Fluent* step_counter_fluent_;

	/* Static fluents' values */
	ValueMap static_fluents_values_;

	/* Minimum transition probability */
	double probability_min_;

	/* Maximum transition probability */
	double probability_max_;

	/* Minimum transition reward */
	double reward_min_;

	/* Maximum transition reward */
	double reward_max_;

	/* Adds artificial fluent values like reward, planner-probability, planner-step-counter ... */
	virtual void add_artificial_values(const TransitionsBlackbox& tb, PddlState& outcome) const;

	/* Removes artificial fluent values like reward, planner-probability, planner-step-counter ... */
	virtual void remove_artificial_values(const TransitionsBlackbox& tb, PddlState& outcome) const;

public :
    /* Constructor */
    BaseDeterminizationWithFluents(const Domain& dm, const Problem& pb, unsigned int plan_length);

    /* Destructor */
    virtual ~BaseDeterminizationWithFluents();

    /* Return the deterministic planner's plan length (command line parameter: trajectories-max-depth) */
    unsigned int get_plan_length() const {return plan_length_;}

    /* Return the probability fluent */
    const Fluent& get_probability_fluent() const {return *probability_fluent_;}

    /* Return the reward fluent */
    const Fluent& get_reward_fluent() const {return *reward_fluent_;}

    /* Return the step counter used to force the deterministic planner to find a plan of a given length (parameter: trajectories-max-depth) */
    const Fluent& get_step_counter_fluent() const {return *step_counter_fluent_;}

    /* Return the static fluents' values */
    const ValueMap& get_static_fluents_values() const {return static_fluents_values_;}

    /* Return the minimum transition probability */
    double get_probability_min() const {return probability_min_;}

    /* Return the maximum transition probability */
    double get_probability_max() const {return probability_max_;}

    /* Return the minimum transition reward */
    double get_reward_min() const {return reward_min_;}

    /* Return the maximum transition reward */
    double get_reward_max() const {return reward_max_;}

    /* Makes a problem */
	virtual void make_problem(const AtomSet& init_atoms, const ValueMap& init_values, const StateFormula& goal_states, const Expression* optimization_expression = NULL);
	
	/* Makes a problem */
	void make_problem(const AtomSet& init_atoms, const ValueMap& init_values, const Expression& optimization_expression);
};

#endif /*BASE_DETERMINIZATION_H_*/
