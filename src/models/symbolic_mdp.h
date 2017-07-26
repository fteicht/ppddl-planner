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

#ifndef SYMBOLIC_MDP_H_
#define SYMBOLIC_MDP_H_

#include <map>

#include "base_mdp.h"
#include "dd_node_ptr.h"

class SymbolicTransition
{
private :
	dd_node_ptr policy_mask_; // BDD
	dd_node_ptr value_mask_; // ADD
	dd_node_ptr probabilities_; // ADD
	dd_node_ptr rewards_; // ADD
	
public :
	SymbolicTransition() {}
	SymbolicTransition(const dd_node_ptr& policy_mask, const dd_node_ptr& value_mask, const dd_node_ptr& probabilities, const dd_node_ptr& rewards);
	void copy(const SymbolicTransition & other);
	
	inline const dd_node_ptr& get_policy_mask() const {return policy_mask_;}
	inline dd_node_ptr& _policy_mask_() {return policy_mask_;}
	
	inline const dd_node_ptr& get_value_mask() const {return value_mask_;}
	inline dd_node_ptr& _value_mask_() {return value_mask_;}
	
	inline const dd_node_ptr& get_probabilities() const {return probabilities_;}
	inline dd_node_ptr& _probabilities_() {return probabilities_;}
	
	inline const dd_node_ptr& get_rewards() const {return rewards_;}
	inline dd_node_ptr& _rewards_() {return rewards_;}
	
	operator bool () const;
	bool operator!() const;
};

class SymbolicMDP : public BaseMDP
{
friend class TransitionsBlackbox;

public :
	typedef dd_node_ptr_associative_container<std::map<const Action*, SymbolicTransition> > transitions_map_t;

protected :
	dd_node_ptr states_; // BDD
	dd_node_ptr unexpanded_states_; // BDD (subset of states_)
	dd_node_ptr goal_states_; // BDD
	transitions_map_t transitions_;
	dd_node_ptr policy_; // ADD
	dd_node_ptr value_function_; // ADD
	dd_node_ptr probability_function_; // ADD
	
	std::map<const Atom*, int> atom_to_index_; // variables indexes
	std::map<CUDD_VALUE_TYPE, const Action*> index_to_action_; // actions indexes
	
	dd_node_ptr unprimed_variables_bdd_cube_; // BDD
	dd_node_ptr unprimed_variables_add_cube_; // ADD
	dd_node_ptr primed_variables_bdd_cube_; // BDD
	dd_node_ptr primed_variables_add_cube_; // ADD

	int * unprimed_to_primed_permutation_;
	int * primed_to_unprimed_permutation_;
	
	std::set<dd_node_ptr*> connected_bdds_; // must be updated when new DD variables are created
	std::set<dd_node_ptr*> connected_adds_; // must be updated when new DD variables are created
	
	dd_node_ptr states_record_;
	dd_node_ptr state_blackbox_;
	virtual void transition_blackbox(const Action& action, double probability, double reward, PddlState* outcome, bool goal);
	void push_pddl_states(int* states_cube, std::list<PddlState>& pddl_states) const;
	
	void get_pddl_state(const dd_node_ptr& cudd_state, PddlState& pddl_state) const;
	dd_node_ptr get_cudd_state(const PddlState& st) const; // throws
	dd_node_ptr get_and_add_cudd_state(const PddlState& st, bool goal); // add new variables if needed
	bool _cudd_state_(const PddlState& pddl_state, dd_node_ptr& cudd_state) const; // returns true if all atoms are known
	void new_dd_variables(const AtomSet& new_atoms, int index);
	dd_node_ptr pick_state(const dd_node_ptr& diagram) const;
	
	void cudd_print_to_ostream(const dd_node_ptr& dd, std::ostream& output_str) const;
	
public :
	SymbolicMDP(const TransitionsBlackbox& tb);
	virtual ~SymbolicMDP();
	
	/* Expands the MDP by constructing the transitions from the given states. Returns the new states */
	dd_node_ptr expand(const dd_node_ptr& states);
	
	/* Returns a random transition (next state and reward transition) for the given state and the given action */
	std::pair<dd_node_ptr, double> random_transition(const dd_node_ptr& st, const Action& action);
	
	inline const dd_node_ptr& get_states() const {return states_;}
	inline dd_node_ptr& _states_() {return states_;}
	
	/* Returns the state formula of the given set of states */
	const StateFormula& state_formula(const dd_node_ptr& states) const;
	
	inline const dd_node_ptr& get_unexpanded_states() const {return unexpanded_states_;}
	inline dd_node_ptr& _unexpanded_states_() {return unexpanded_states_;}
	
	inline const dd_node_ptr& get_goal_states() const {return goal_states_;}
	inline dd_node_ptr& _goal_states_() {return goal_states_;}
	
	inline const transitions_map_t& get_transitions() const {return transitions_;}
	inline transitions_map_t& _transitions_() {return transitions_;}
	
	dd_node_ptr add_state(const PddlState& st);
	void remove_state(const PddlState& st);
	dd_node_ptr get_state(const PddlState& st) const; // throws
	
	inline const dd_node_ptr& get_policy() const {return policy_;}
	inline dd_node_ptr& _policy_() {return policy_;}
	
	inline const dd_node_ptr& get_value_function() const {return value_function_;}
	inline dd_node_ptr& _value_function_() {return value_function_;}

	inline const dd_node_ptr& get_probability_function() const {return probability_function_;}
	inline dd_node_ptr& _probability_function_() {return probability_function_;}
	
	/* Returns the policy of the given state */
	virtual const Action& get_policy(const PddlState& st) const; // throws
	
	/* Returns the policy of the given state */
	virtual const Action& get_policy(const dd_node_ptr& st) const; // throws
	
	/* Returns the value of the given state */
	virtual double get_value(const PddlState& st) const; // throws
	
	/* Returns the value of the given state */
	virtual double get_value(const dd_node_ptr& st) const; // throws

	/* Returns the probability of the given state */
	virtual double get_probability(const PddlState& st) const; // throws
	
	/* Returns the probability of the given state */
	virtual double get_probability(const dd_node_ptr& st) const; // throws
	
	/* Returns true if the given state has been explored */
	virtual bool explored(const PddlState& st) const;
	
	/* Returns the set of explored states among the given states */
	dd_node_ptr explored(const dd_node_ptr& states) const;
	
	/* Returns true if the given state has been expanded */
	virtual bool expanded(const PddlState& st) const;
	
	/* Returns the set of expanded states among the given states */
	dd_node_ptr expanded(const dd_node_ptr& states) const;
	
	/* Returns true if the given state is a goal (among explored states) */
	virtual bool goal(const PddlState& st) const;
	
	/* Returns the set of goal states among the given states (among explored states) */
	dd_node_ptr goals(const dd_node_ptr& states) const;
	
	/* Returns the set of states among given states where the policy is defined */
	dd_node_ptr policy_defined(const dd_node_ptr& states) const;
	
	/* Decomposes a set of states into a list of PDDL states */
	void decompose(const dd_node_ptr& cudd_states, std::list<PddlState>& pddl_states) const;
	
	/* Prints the MDP model */
	virtual std::string print_model() const;
	
	/* Prints the MDP solution */
	virtual std::string print_solution() const;
	
	inline const dd_node_ptr& get_unprimed_variables_bdd_cube() const {return unprimed_variables_bdd_cube_;}
	inline const dd_node_ptr& get_unprimed_variables_add_cube() const {return unprimed_variables_add_cube_;}
	inline const dd_node_ptr& get_primed_variables_bdd_cube() const {return primed_variables_bdd_cube_;}
	inline const dd_node_ptr& get_primed_variables_add_cube() const {return primed_variables_add_cube_;}
	
	inline int* get_unprimed_to_primed_permutation() const {return unprimed_to_primed_permutation_;}
	inline int* get_primed_to_unprimed_permutation() const {return primed_to_unprimed_permutation_;}
	
	inline const std::map<const Atom*, int>& get_atom_to_index() const {return atom_to_index_;}
	inline const std::map<CUDD_VALUE_TYPE, const Action*>& get_index_to_action() const {return index_to_action_;}
	
	/* Connects a BDD which must be updated when new DD variables are created */
	void connect_bdd(dd_node_ptr& bdd) {connected_bdds_.insert(&bdd);}
	
	/* Disconnects a BDD which must be updated when new DD variables are created */
	void disconnect_bdd(dd_node_ptr& bdd) {connected_bdds_.erase(&bdd);}
	
	/* Connects an ADD which must be updated when new DD variables are created */
	void connect_add(dd_node_ptr& add) {connected_adds_.insert(&add);}
	
	/* Disconnects an ADD which must be updated when new DD variables are created */
	void disconnect_add(dd_node_ptr& add) {connected_adds_.erase(&add);}
};


class ScopedConnectionDiagram
{
protected :
	dd_node_ptr& diagram_;
	static SymbolicMDP* mdp_;
	
public :
	ScopedConnectionDiagram(dd_node_ptr& diagram) : diagram_(diagram) {}
	virtual ~ScopedConnectionDiagram() {}
	
	static inline void set_mdp(SymbolicMDP* mdp) {mdp_ = mdp;}
	static inline const SymbolicMDP* get_mdp() {return mdp_;}
	
	virtual bool is_bdd() const =0;
	virtual bool is_add() const =0;
};


class ScopedConnectionBDD : public ScopedConnectionDiagram
{	
public :
	ScopedConnectionBDD(dd_node_ptr& bdd) : ScopedConnectionDiagram(bdd) {mdp_->connect_bdd(diagram_);}
	virtual ~ScopedConnectionBDD() {mdp_->disconnect_bdd(diagram_);}
	
	virtual bool is_bdd() const {return true;}
	virtual bool is_add() const {return false;}
};


class ScopedConnectionADD : public ScopedConnectionDiagram
{
public :
	ScopedConnectionADD(dd_node_ptr& add) : ScopedConnectionDiagram(add) {mdp_->connect_add(diagram_);}
	virtual ~ScopedConnectionADD() {mdp_->disconnect_add(diagram_);}
	
	virtual bool is_bdd() const {return false;}
	virtual bool is_add() const {return true;}
};

#endif /*SYMBOLIC_MDP_H_*/
