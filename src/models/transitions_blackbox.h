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

#ifndef TRANSITIONS_BLACKBOX_H_
#define TRANSITIONS_BLACKBOX_H_

#include <ppddl_planner_mdpsim/formulas.h>
#include <ppddl_planner_mdpsim/expressions.h>
#include <ppddl_planner_mdpsim/actions.h>
#include <ppddl_planner_mdpsim/problems.h>
#include <list>
#include <set>

class TransitionsBlackbox;

class PddlState
{
private :
	/* Atomic state formulas that hold in this state. */
	AtomSet atoms_;
	/* Fluent values in this state. */
	ValueMap values_;

	/* TransitionsBlackbox */
	static const TransitionsBlackbox* tb_;

public :
	/* Constructs an empty state */
	inline PddlState() {}

	/* Constructs a state with the set of atoms that hold in this state */
	PddlState(const AtomSet& atoms, const ValueMap& values);

	/* Returns a copy of this state (without fluents check). The new state must be manually deleted. */
	PddlState* copy() const;

	/* Sets the set of atoms that hold in this state. */
	void set_atoms(const AtomSet& atoms) {atoms_ = atoms;}

	/* Returns the set of atoms that hold in this state. */
	inline const AtomSet& atoms() const {return atoms_;}
	inline AtomSet& _atoms_() {return atoms_;}

	/* Sets the fluent values for this state. */
	void set_values(const ValueMap& values);
	void copy_values(const PddlState& st) {this->values_ = st.values_;} // without fluents check

	/* Returns the fluent values for this state. */
	inline const ValueMap& values() const {return values_;}
	inline ValueMap& _values_() {return values_;}

	/* Returns the state formula of this state */
	const StateFormula& state_formula() const;

	/* Sets the transition blackbox */
	inline static void set_transitions_blackbox(const TransitionsBlackbox* tb) {tb_ = tb;}

	/* Returns the transition blackbox */
	inline static const TransitionsBlackbox* get_transitions_blackbox() {return tb_;}
};

std::ostream& operator<<(std::ostream& os, const PddlState& s);

struct PddlStateLess : public std::binary_function<const PddlState*, const PddlState*, bool>
{
	bool operator()(const PddlState* s1, const PddlState* s2) const;
};

/*
 * Set of PDDL states
 */
struct PddlStateSet : public std::set<const PddlState*, PddlStateLess> {
};


/*
 * Set of updates.
 */
//struct UpdateSet : public std::set<const Update*> {
//};

class TransitionsBlackbox;

class TransitionsBlackboxUser
{
friend class TransitionsBlackbox;

protected :
	const TransitionsBlackbox& tb_;
	virtual void transition_blackbox(const Action& action, double probability, double reward, PddlState* outcome, bool goal) =0;

public :
	TransitionsBlackboxUser(const TransitionsBlackbox& tb) : tb_(tb) {}
	virtual ~TransitionsBlackboxUser() {}

	inline const TransitionsBlackbox& get_transitions_blackbox() const {return tb_;}
};


class TransitionsBlackbox
{
public :
	struct PddlTransition
	{
		double probability_;
		AtomSet adds_;
		AtomSet deletes_;
		UpdateSet updates_;
		PddlTransition() {}
		PddlTransition(double probability, AtomSet adds, AtomSet deletes, UpdateSet updates)
			: probability_(probability), adds_(adds), deletes_(deletes), updates_(updates) {}
	};
	typedef std::list<PddlTransition> transition_list_t;

	typedef enum
	{
		METRIC_REWARD,
		METRIC_GOAL
	}
	metric_t;

private :
	const Problem& problem_;
	const Fluent* reward_fluent_;
	const Fluent* total_time_fluent_;
	const Fluent* goal_achieved_fluent_;
	const Action* planner_stay_action_;
	metric_t metric_type_;

	void effect_transitions(const Effect& effect, const PddlState& income, transition_list_t& transitions) const;
	void add_effect_transitions(const AddEffect& add_effect, const PddlState& income, transition_list_t& transitions) const;
	void delete_effect_transitions(const DeleteEffect& delete_effect, const PddlState& income, transition_list_t& transitions) const;
	void update_effect_transitions(const UpdateEffect& update_effect, const PddlState& income, transition_list_t& transitions) const;
	void conjunctive_effect_transitions(const ConjunctiveEffect& conjunctive_effect, const PddlState& income, transition_list_t& transitions) const;
	void conditional_effect_transitions(const ConditionalEffect& conditional_effect, const PddlState& income, transition_list_t& transitions) const;
	void probabilistic_effect_transitions(const ProbabilisticEffect& probabilistic_effect, const PddlState& income, transition_list_t& transitions) const;
	void quantified_effect_transitions(const QuantifiedEffect& quantified_effect, const PddlState& income, transition_list_t& transitions) const;

	void implicit_to_explicit(const PddlState& income, const AtomSet& adds, const AtomSet& deletes, const UpdateSet& updates, PddlState& outcome, double& reward, bool& goal) const;
	//void compute_outcomes(const PddlState& income, const Action& action, const transition_list_t& pddl_transitions, TransitionsBlackboxUser& tbu) const; // throws
	void compute_outcomes(const PddlState& income, const Action& action, const Effect::transition_list_t& pddl_transitions, TransitionsBlackboxUser& tbu) const; // throws
	void stay_transition(const PddlState& income, TransitionsBlackboxUser& tbu) const;

public :
	TransitionsBlackbox(const Problem& problem, metric_t metric_type = METRIC_REWARD);
	~TransitionsBlackbox();

	/* Generates a random transition for a given state and a given action. The outcome state is referenced. */
	void random_transition(const PddlState& income, const Action& action, PddlState& outcome, double& reward, bool& goal) const;

	/* Generates all the transitions for a given state */
	void transitions(const PddlState& income, TransitionsBlackboxUser& tbu) const;

	/* Returns the problem */
	const Problem& get_problem() const {return problem_;}

	/* Returns the reward function if defined, else NULL */
	inline const Fluent* reward_fluent() const {return reward_fluent_;}

	/* Returns the total-time function if defined, else NULL */
	inline const Fluent* total_time_fluent() const {return total_time_fluent_;}

	/* Returns the goal-achieved function if defined, else NULL */
	inline const Fluent* goal_achieved_fluent() const {return goal_achieved_fluent_;}

	/* Sets the metric */
	inline void set_metric(metric_t metric_type) {metric_type_ = metric_type;}

	/* Gets the metric */
	inline metric_t get_metric() const {return metric_type_;}

	/* Gets the "planner-stay-action" */
	inline const Action& get_planner_stay_action() const {return *planner_stay_action_;}
};

#endif /*TRANSITIONS_BLACKBOX_H_*/
