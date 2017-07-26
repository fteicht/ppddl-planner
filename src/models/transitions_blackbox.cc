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

#include <sstream>
#include <algorithm>
#include <typeinfo>

#include "transitions_blackbox.h"
#include "base_mdp.h"
#include "commons/base_exceptions.h"


// CLASS PddlState

const TransitionsBlackbox* PddlState::tb_ = NULL;

PddlState::PddlState(const AtomSet& atoms, const ValueMap& values)
: atoms_(atoms)
{
	if ((tb_->reward_fluent()) || (tb_->total_time_fluent()) || (tb_->goal_achieved_fluent()))
		set_values(values);
	else
		values_ = values;
}


PddlState* PddlState::copy() const
{
	PddlState* state_copy = new PddlState();
	state_copy->atoms_ = this->atoms_;
	state_copy->values_ = this->values_;
	return state_copy;
}


void PddlState::set_values(const ValueMap& values)
{
	values_.clear();

	for (ValueMap::const_iterator Ivm = values.begin() ; Ivm != values.end() ; ++Ivm)
	{
		if (((Ivm->first) != (tb_->reward_fluent()))
				&& ((Ivm->first) != (tb_->total_time_fluent()))
				&& ((Ivm->first) != (tb_->goal_achieved_fluent())))
			values_.insert(std::make_pair(Ivm->first, Ivm->second));
	}
}


const StateFormula& PddlState::state_formula() const
{
	const StateFormula* res = &StateFormula::TRUE;

	for (Atom::AtomTable::const_iterator ai = Atom::atom_table().begin() ; ai != Atom::atom_table().end() ; ++ai)
	{
		if (!PredicateTable::static_predicate((*ai)->predicate()))
		{
			if (atoms_.find(*ai) != atoms_.end())
				res = &((*res) && (**ai));
			else
				res = &((*res) && Negation::make(**ai));
		}
	}

	for (ValueMap::const_iterator vi = values_.begin() ; vi != values_.end() ; vi++)
		res = &((*res) && EqualTo::make(*(vi->first), Value(vi->second)));

	return (*res);
}


std::ostream& operator<<(std::ostream& os, const PddlState& s)
{
	bool first = true;

	for (AtomSet::const_iterator ai = s.atoms().begin(); ai != s.atoms().end(); ai++)
	{
		const Atom& atom = **ai;

		if (!PredicateTable::static_predicate(atom.predicate()))
		{
			if (first)
				first = false;
			else
				os << ' ';

			os << atom;
		}
	}

	for (ValueMap::const_iterator vi = s.values().begin(); vi != s.values().end(); vi++)
	{
		const Fluent& fluent = *(*vi).first;

		if (first)
			first = false;
		else
			os << ' ';

		os << "(= " << fluent << ' ' << (*vi).second << ")";
	}

	return os;
}


bool PddlStateLess::operator ()(const PddlState* s1, const PddlState* s2) const
{
	AtomSet::const_iterator Ias1 = s1->atoms().begin();

	for (AtomSet::const_iterator Ias2 = s2->atoms().begin() ; Ias2 != s2->atoms().end() ; ++Ias2)
	{
		if (Ias1 == s1->atoms().end())
			return true;

		if (((*Ias1)->predicate()) < ((*Ias2)->predicate()))
			return true;

		if (((*Ias2)->predicate()) < ((*Ias1)->predicate()))
			return false;

		if (((*Ias1)->terms()) < ((*Ias2)->terms()))
			return true;

		if (((*Ias2)->terms()) < ((*Ias1)->terms()))
			return false;

		++Ias1;
	}

	if (Ias1 != s1->atoms().end())
		return false;

	ValueMap::const_iterator Ivs1 = s1->values().begin();

	for (ValueMap::const_iterator Ivs2 = s2->values().begin() ; Ivs2 != s2->values().end() ; ++Ivs2)
	{
		if (Ivs1 == s1->values().end())
			return true;

		if ((Ivs1->first->function()) < (Ivs2->first->function()))
			return true;

		if ((Ivs2->first->function()) < (Ivs1->first->function()))
			return false;

		if ((Ivs1->first->terms()) < (Ivs2->first->terms()))
			return true;

		if ((Ivs2->first->terms()) < (Ivs1->first->terms()))
			return false;

		if ((Ivs1->second) < (Ivs2->second))
			return true;

		if ((Ivs1->second) > (Ivs2->second))
			return false;

		++Ivs1;
	}

	if (Ivs1 != s1->values().end())
		return false;

	return false;
}


// CLASS TransitionsBlackbox

TransitionsBlackbox::TransitionsBlackbox(const Problem& problem, metric_t metric_type)
: problem_(problem), reward_fluent_(NULL), total_time_fluent_(NULL), goal_achieved_fluent_(NULL)
{
	const Function* reward_function = problem_.domain().functions().find_function("reward");

	if (reward_function)
		reward_fluent_ = Fluent::fluent(*reward_function);

	const Function* total_time_function = problem_.domain().functions().find_function("total-time");

	if (total_time_function)
		total_time_fluent_ = Fluent::fluent(*total_time_function);

	const Function* goal_achieved_function = problem_.domain().functions().find_function("goal-achieved");

	if (goal_achieved_function)
		goal_achieved_fluent_ = Fluent::fluent(*goal_achieved_function);

	planner_stay_action_ = new Action("planner-stay-action");
	PddlState::set_transitions_blackbox(this);
}


TransitionsBlackbox::~TransitionsBlackbox()
{
	delete planner_stay_action_;

	if (PddlState::get_transitions_blackbox() == this)
		PddlState::set_transitions_blackbox(NULL);
}


void TransitionsBlackbox::random_transition(const PddlState& income, const Action& action, PddlState& outcome, double& reward, bool& goal) const
{
	AtomList _adds;
	AtomList _deletes;
	UpdateList _updates;
	action.effect().state_change(_adds, _deletes, _updates, problem_.terms(), income.atoms(), income.values());
	AtomSet adds;
	std::copy(_adds.begin(), _adds.end(), std::inserter(adds, adds.begin()));
	AtomSet deletes;
	std::copy(_deletes.begin(), _deletes.end(), std::inserter(deletes, deletes.begin()));
	UpdateSet updates;
	std::copy(_updates.begin(), _updates.end(), std::inserter(updates, updates.begin()));
	implicit_to_explicit(income, adds, deletes, updates, outcome, reward, goal);
}


void TransitionsBlackbox::transitions(const PddlState& income, TransitionsBlackboxUser& tbu) const
{
	ActionList actions;
	problem_.enabled_actions(actions, income.atoms(), income.values());

//	if (actions.empty())
//	{
//		std::ostringstream str;
//		str << "state " << income << " : ";
//		throw BaseException(str.str() + "no applicable actions", "TransitionsBlackbox::transitions");
//	}

	if (actions.empty() || problem_.goal().holds(problem_.terms(), income.atoms(), income.values()))
    {
		stay_transition(income, tbu);
        return;
    }

	for (ActionList::const_iterator Iac = actions.begin() ; Iac != actions.end() ; ++Iac)
	{
		try
		{
			Effect::transition_list_t transitions;
			//effect_transitions((*Iac)->effect(), income, transitions);
			(*Iac)->effect().transitions(problem_.terms(), income.atoms(), income.values(), transitions);
			compute_outcomes(income, **Iac, transitions, tbu);
		}
		catch (BaseException& error)
		{
			std::string err_str = "action " + ((*Iac)->name()) + " : ";
			BaseException new_error(err_str + error.what(), "TransitionsBlackbox::transitions");
			throw new_error;
		}
	}
}


void TransitionsBlackbox::effect_transitions(const Effect& effect, const PddlState& income, transition_list_t& transitions) const
{
	if (effect.empty())
	{
		transitions.push_back(PddlTransition(1.0, AtomSet(), AtomSet(), UpdateSet()));
		return;
	}

	const AddEffect* add_effect = dynamic_cast<const AddEffect*>(&effect);

	if (add_effect)
	{
		add_effect_transitions(*add_effect, income, transitions);
		return;
	}

	const DeleteEffect* delete_effect = dynamic_cast<const DeleteEffect*>(&effect);

	if (delete_effect)
	{
		delete_effect_transitions(*delete_effect, income, transitions);
		return;
	}

	const UpdateEffect* update_effect = dynamic_cast<const UpdateEffect*>(&effect);

	if (update_effect)
	{
		update_effect_transitions(*update_effect, income, transitions);
		return;
	}

	const ConjunctiveEffect* conjunctive_effect = dynamic_cast<const ConjunctiveEffect*>(&effect);

	if (conjunctive_effect)
	{
		conjunctive_effect_transitions(*conjunctive_effect, income, transitions);
		return;
	}

	const ConditionalEffect* conditional_effect = dynamic_cast<const ConditionalEffect*>(&effect);

	if (conditional_effect)
	{
		conditional_effect_transitions(*conditional_effect, income, transitions);
		return;
	}

	const ProbabilisticEffect* probabilistic_effect = dynamic_cast<const ProbabilisticEffect*>(&effect);

	if (probabilistic_effect)
	{
		probabilistic_effect_transitions(*probabilistic_effect, income, transitions);
		return;
	}

	const QuantifiedEffect* quantified_effect = dynamic_cast<const QuantifiedEffect*>(&effect);

	if (quantified_effect)
	{
		quantified_effect_transitions(*quantified_effect, income, transitions);
		return;
	}

	/*
	 * No other types of effects exist.
	 */
	std::ostringstream str;
	str << "state " << income << " : unexpected effect " << typeid(effect).name();
	throw BaseException(str.str(), "TransitionsBlackbox::effect_transitions");
}


void TransitionsBlackbox::add_effect_transitions(const AddEffect& add_effect, const PddlState& income, transition_list_t& transitions) const
{
	transitions.push_back(PddlTransition());
	transitions.back().probability_ = 1.0;
	transitions.back().adds_.insert(&(add_effect.atom()));
}


void TransitionsBlackbox::delete_effect_transitions(const DeleteEffect& delete_effect, const PddlState& income, transition_list_t& transitions) const
{
	transitions.push_back(PddlTransition());
	transitions.back().probability_ = 1.0;
	transitions.back().deletes_.insert(&(delete_effect.atom()));
}


void TransitionsBlackbox::update_effect_transitions(const UpdateEffect& update_effect, const PddlState& income, transition_list_t& transitions) const
{
	transitions.push_back(PddlTransition());
	transitions.back().probability_ = 1.0;
	transitions.back().updates_.insert(&(update_effect.update()));
}


void TransitionsBlackbox::conjunctive_effect_transitions(const ConjunctiveEffect& conjunctive_effect, const PddlState& income, transition_list_t& transitions) const
{
	transitions.push_back(PddlTransition(1.0, AtomSet(), AtomSet(), UpdateSet()));

	for (EffectList::const_iterator Ieff = conjunctive_effect.conjuncts().begin() ; Ieff != conjunctive_effect.conjuncts().end() ; ++Ieff)
	{
		transition_list_t transitions_temp;
		effect_transitions(**Ieff, income, transitions_temp);
		transition_list_t transitions_temp_bis;

		for (transition_list_t::const_iterator Itr1 = transitions.begin() ; Itr1 != transitions.end() ; ++Itr1)
		{
			for (transition_list_t::const_iterator Itr2 = transitions_temp.begin() ; Itr2 != transitions_temp.end() ; ++Itr2)
			{
				transitions_temp_bis.push_back(PddlTransition());
				transitions_temp_bis.back().probability_ = (Itr1->probability_) * (Itr2->probability_);

				std::set_union(Itr1->adds_.begin(), Itr1->adds_.end(),
						Itr2->adds_.begin(), Itr2->adds_.end(),
						std::inserter(transitions_temp_bis.back().adds_, transitions_temp_bis.back().adds_.begin()));

				std::set_union(Itr1->deletes_.begin(), Itr1->deletes_.end(),
						Itr2->deletes_.begin(), Itr2->deletes_.end(),
						std::inserter(transitions_temp_bis.back().deletes_, transitions_temp_bis.back().deletes_.begin()));

				std::set_union(Itr1->updates_.begin(), Itr1->updates_.end(),
						Itr2->updates_.begin(), Itr2->updates_.end(),
						std::inserter(transitions_temp_bis.back().updates_, transitions_temp_bis.back().updates_.begin()));
			}
		}

		transitions = transitions_temp_bis;
	}
}


void TransitionsBlackbox::conditional_effect_transitions(const ConditionalEffect& conditional_effect, const PddlState& income, transition_list_t& transitions) const
{
	if (conditional_effect.condition().holds(problem_.terms(), income.atoms(), income.values()))
		effect_transitions(conditional_effect.effect(), income, transitions);
	else
		transitions.push_back(PddlTransition(1.0, AtomSet(), AtomSet(), UpdateSet()));
}


void TransitionsBlackbox::probabilistic_effect_transitions(const ProbabilisticEffect& probabilistic_effect, const PddlState& income, transition_list_t& transitions) const
{
	double prob_sum = 0.0;

	for (size_t eff = 0 ; eff < probabilistic_effect.size() ; eff++)
	{
		transition_list_t transitions_temp;
		effect_transitions(probabilistic_effect.effect(eff), income, transitions_temp);
		double prob = probabilistic_effect.probability(eff).double_value();
		prob_sum += prob;

		for (transition_list_t::const_iterator Itr = transitions_temp.begin() ; Itr != transitions_temp.end() ; ++Itr)
			transitions.push_back(PddlTransition(prob * (Itr->probability_), Itr->adds_, Itr->deletes_, Itr->updates_));
	}

	if (prob_sum < 1.0)
		transitions.push_back(PddlTransition(1.0 - prob_sum, AtomSet(), AtomSet(), UpdateSet()));
}


void TransitionsBlackbox::quantified_effect_transitions(const QuantifiedEffect& quantified_effect, const PddlState& income, transition_list_t& transitions) const
{
	// Never reached since the instantiation of a quantified effect transforms it into a conjunctive effect
	throw BaseException("quantified effects should be transformed into conjunctive effects after instantiation", "TransitionsBlackbox::quantified_effect_transitions");
}


//void TransitionsBlackbox::compute_outcomes(const PddlState& income, const Action& action, const transition_list_t& implicit_transitions, TransitionsBlackboxUser& tbu) const
void TransitionsBlackbox::compute_outcomes(const PddlState& income, const Action& action, const Effect::transition_list_t& implicit_transitions, TransitionsBlackboxUser& tbu) const
{
	double prob_sum = 0.0;

	//for (transition_list_t::const_iterator Itri = implicit_transitions.begin() ; Itri != implicit_transitions.end() ; ++Itri)
	for (Effect::transition_list_t::const_iterator Itri = implicit_transitions.begin() ; Itri != implicit_transitions.end() ; ++Itri)
	{
		prob_sum += Itri->probability_;
		double reward;
		bool goal;
		PddlState* outcome = new PddlState();
		implicit_to_explicit(income, Itri->adds_, Itri->deletes_, Itri->updates_, *outcome, reward, goal);
		tbu.transition_blackbox(action, Itri->probability_, reward, outcome, goal);
	}

	/*
	if ((prob_sum < 1.0) || (prob_sum > 1.0))
	{
		std::ostringstream str;
		str << "action " << action << " in state " << income << " : ";
		std::ostringstream prob_str;
		prob_str << prob_sum;
		throw BaseException(str.str() + "incomplete effect (probability sum is " + prob_str.str() + ")", "TransitionsBlackbox::compute_outcomes");
	}
	*/
}


void TransitionsBlackbox::implicit_to_explicit(const PddlState& income, const AtomSet& adds, const AtomSet& deletes, const UpdateSet& updates, PddlState& outcome, double& reward, bool& goal) const
{
	AtomSet temp_atoms;
	std::set_difference(income.atoms().begin(), income.atoms().end(),
			deletes.begin(), deletes.end(),
			std::inserter(temp_atoms, temp_atoms.begin()));
	std::set_union(temp_atoms.begin(), temp_atoms.end(),
			adds.begin(), adds.end(),
			std::inserter(outcome._atoms_(), outcome._atoms_().begin()));

	outcome.copy_values(income);
	ValueMap::iterator Irw = outcome._values_().end();

	if (reward_fluent_)
		Irw = outcome._values_().insert(std::make_pair(reward_fluent_, Rational(0))).first;

	if (total_time_fluent_)
		outcome._values_().insert(std::make_pair(total_time_fluent_, Rational(0))).first;

	if (goal_achieved_fluent_)
		outcome._values_().insert(std::make_pair(goal_achieved_fluent_, Rational(0))).first;

	for (UpdateSet::const_iterator Iup = updates.begin() ; Iup != updates.end() ; ++Iup)
		(*Iup)->affect(outcome._values_());

	goal = problem_.goal().holds(problem_.terms(), outcome.atoms(), outcome.values());

	switch (metric_type_)
	{
		case METRIC_REWARD :
		{
			if (Irw != outcome._values_().end())
			{
				if (goal && (problem_.goal_reward()))
					problem_.goal_reward()->affect(outcome._values_());

				reward = Irw->second.double_value();
				outcome._values_().erase(Irw);
			}
			else
				reward = 0.0;

			break;
		}

		case METRIC_GOAL :
		{
			if (goal)
				reward = 0.0;
			else
				reward = -1.0;

			if (Irw != outcome._values_().end())
				outcome._values_().erase(Irw);

			break;
		}
	}

	if (total_time_fluent_)
		outcome._values_().erase(total_time_fluent_);

	if (goal_achieved_fluent_)
		outcome._values_().erase(goal_achieved_fluent_);
}


void TransitionsBlackbox::stay_transition(const PddlState& income, TransitionsBlackboxUser& tbu) const
{
	PddlState* outcome = income.copy();

	if (reward_fluent_)
		outcome->_values_().insert(std::make_pair(reward_fluent_, Rational(0))).first;

	if (total_time_fluent_)
		outcome->_values_().insert(std::make_pair(total_time_fluent_, Rational(0))).first;

	if (goal_achieved_fluent_)
		outcome->_values_().insert(std::make_pair(goal_achieved_fluent_, Rational(0))).first;

	bool goal = problem_.goal().holds(problem_.terms(), outcome->atoms(), outcome->values());

	if (reward_fluent_)
		outcome->_values_().erase(reward_fluent_);

	if (total_time_fluent_)
		outcome->_values_().erase(total_time_fluent_);

	if (goal_achieved_fluent_)
		outcome->_values_().erase(goal_achieved_fluent_);

	if ((metric_type_ == METRIC_GOAL) && (!goal))
		tbu.transition_blackbox(*planner_stay_action_, 1.0, -1.0, outcome, goal);
	else
		tbu.transition_blackbox(*planner_stay_action_, 1.0, 0.0, outcome, goal);
}
