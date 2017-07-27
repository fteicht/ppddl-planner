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

#include <unistd.h>
#include <fstream>
#include <sstream>
#include <limits>
#include <algorithm>
#include <mdpsim/domains.h>
#include <mdpsim/problems.h>

#include "base_determinization.h"
#include "transitions_blackbox.h"
#include "commons/base_exceptions.h"

BaseDeterminization::BaseDeterminization(const Domain& dm, const Problem& pb)
: domain_(dm), problem_(pb)
{
	domain_file_name_ = std::string("ppddl_planner_deterministic_domain_") + dm.name() + ".pddl";
	problem_file_name_ = std::string("ppddl_planner_deterministic_problem_") + pb.name() + ".pddl";
}


BaseDeterminization::~BaseDeterminization()
{
	for (ActionSchemaMap::iterator Ia = deterministic_action_schemas_.begin() ; Ia != deterministic_action_schemas_.end() ; ++Ia)
		delete (Ia->second);

	for (ActionSet::iterator Ia = deterministic_actions_.begin() ; Ia != deterministic_actions_.end() ; ++Ia)
		delete (*Ia);
}


void BaseDeterminization::make_domain()
{
	generate_deterministic_actions();

	char dirpath[dirpath_length_];
	getcwd(dirpath, dirpath_length_);
	chdir ("/tmp/");
	std::ofstream cdf(domain_file_name_.c_str(), std::ios_base::out | std::ios_base::trunc);

	cdf << "(define (domain ppddl_planner_deterministic_" << domain_.name() << ")" << std::endl;
	write_requirements(cdf);
	write_types(cdf);
	write_constants(cdf);
	write_predicates(cdf);
	write_functions(cdf);
	write_actions(cdf);
	cdf << ")" << std::endl;

	cdf.close();
	chdir(dirpath);
}


void BaseDeterminization::add_deterministic_action_schema(const ActionSchema& probabilistic_action, const std::string& deterministic_name, const Effect& deterministic_effect)
{
	ActionSchema* action_schema = new ActionSchema(deterministic_name);

	for (VariableList::const_iterator Iv = probabilistic_action.parameters().begin() ; Iv != probabilistic_action.parameters().end() ; ++Iv)
		action_schema->add_parameter(*Iv);

	action_schema->set_precondition(probabilistic_action.precondition());
	action_schema->set_effect(deterministic_effect);
	action_schema->instantiations(deterministic_actions_, problem_.terms(), problem_.init_atoms(), problem_.init_values());
	deterministic_action_schemas_.insert(std::make_pair(deterministic_name, action_schema));
}


void BaseDeterminization::write_requirements(std::ofstream& cdf) const
{
	cdf << "  (:requirements";

	if (domain_.requirements.strips && domain_.requirements.typing && domain_.requirements.equality
			&& domain_.requirements.negative_preconditions && domain_.requirements.disjunctive_preconditions
			&& domain_.requirements.existential_preconditions && domain_.requirements.universal_preconditions
			&& domain_.requirements.conditional_effects)
		cdf << " :adl";
	else
	{
		if (domain_.requirements.strips) {cdf << " :strips";}
		if (domain_.requirements.typing) {cdf << " :typing";}
		if (domain_.requirements.equality) {cdf << " :equality";}
		if (domain_.requirements.negative_preconditions) {cdf << " :negative-preconditions";}
		if (domain_.requirements.disjunctive_preconditions) {cdf << " :disjunctive-preconditions";}

		if (domain_.requirements.existential_preconditions && domain_.requirements.universal_preconditions)
			cdf << " :quantified-preconditions";
		else
		{
			if (domain_.requirements.existential_preconditions) {cdf << " :existential-preconditions";}
			if (domain_.requirements.universal_preconditions) {cdf << " :universal-preconditions";}
		}

		if (domain_.requirements.conditional_effects) {cdf << " :conditional-effects";}
	}
	if ((domain_.requirements.fluents) || dynamic_cast<const BaseDeterminizationWithFluents*>(this) != NULL) {
        cdf << " :fluents";
    }
	cdf << ")" << std::endl;
}


void BaseDeterminization::write_types(std::ofstream& cdf) const
{
	if (!domain_.requirements.typing) // type object allowed by default
		return;

	std::map<std::string, std::set<std::string> > types_subtypes_map;
	std::set<std::string> top_types;
	collect_types(types_subtypes_map, top_types);

	if (types_subtypes_map.empty() && top_types.empty())
        return;

	cdf << "  (:types";
	_write_types_(cdf, "object", top_types, types_subtypes_map);
	cdf << ")" << std::endl;
}


void BaseDeterminization::collect_types(std::map<std::string, std::set<std::string> >& types_subtypes_map, std::set<std::string>& top_types) const
{
	for (std::map<std::string, Type>::const_iterator It1 = domain_.types().get_types().begin() ; It1 != domain_.types().get_types().end() ; ++It1)
	{
		if (It1->second.simple())
		{
			std::set<std::string>& subtypes_map = types_subtypes_map[It1->first];
			bool top_tp = true;

			for (std::map<std::string, Type>::const_iterator It2 = domain_.types().get_types().begin() ; It2 != domain_.types().get_types().end() ; ++It2)
			{
				if ((It2->second.simple()) && ((It1->second) != (It2->second)))
				{
					if (TypeTable::subtype(It2->second, It1->second))
						subtypes_map.insert(It2->first);
					else if (top_tp && TypeTable::subtype(It1->second, It2->second))
						top_tp = false;
				}
			}

			if (top_tp)
				top_types.insert(It1->first);
		}
	}
}


void BaseDeterminization::_write_types_(std::ofstream& cdf, const std::string& top_type, const std::set<std::string>& sub_types, const std::map<std::string, std::set<std::string> >& types_subtypes_map) const
{
	for (std::set<std::string>::const_iterator Ist = sub_types.begin() ; Ist != sub_types.end() ; ++Ist)
		cdf << " " << (*Ist);

	cdf << " - " << top_type;

	for (std::set<std::string>::const_iterator Ist = sub_types.begin() ; Ist != sub_types.end() ; ++Ist)
	{
		const std::set<std::string>& types_subtypes = types_subtypes_map.find(*Ist)->second;

		if (!types_subtypes.empty())
			_write_types_(cdf, *Ist, types_subtypes, types_subtypes_map);
	}
}


void BaseDeterminization::write_constants(std::ofstream& cdf) const
{
	cdf << "  (:constants";

	if (domain_.requirements.typing)
	{
		for (std::map<std::string, Object>::const_iterator Iob = domain_.terms().get_objects().begin() ; Iob != domain_.terms().get_objects().end() ; ++Iob)
			cdf << " " << (Iob->first) << " - " << TermTable::type(Iob->second);
	}
	else
	{
		for (std::map<std::string, Object>::const_iterator Iob = domain_.terms().get_objects().begin() ; Iob != domain_.terms().get_objects().end() ; ++Iob)
			cdf << " " << (Iob->first);
	}

	cdf << ")" << std::endl;
}


void BaseDeterminization::write_predicates(std::ofstream& cdf) const
{
	cdf << "  (:predicates" << std::endl;

	for (std::map<std::string, Predicate>::const_iterator Ipr = domain_.predicates().get_predicates().begin() ; Ipr != domain_.predicates().get_predicates().end() ; ++Ipr)
	{
		cdf << "    (" << (Ipr->first);
		const TypeList& predicate_types = PredicateTable::parameters(Ipr->second);
		unsigned int cnt = 1;

		if (domain_.requirements.typing)
		{
			for (TypeList::const_iterator It = predicate_types.begin() ; It != predicate_types.end() ; ++It)
			{
				cdf << " ?v" << cnt << " - " << (*It);
				cnt++;
			}
		}
		else
		{
			for (TypeList::const_iterator It = predicate_types.begin() ; It != predicate_types.end() ; ++It)
			{
				cdf << " ?v" << cnt;
				cnt++;
			}
		}

		cdf << ")" << std::endl;
	}

	cdf << "  )" << std::endl;
}


void BaseDeterminization::write_functions(std::ofstream& cdf) const
{
	if ((!domain_.requirements.fluents) && (dynamic_cast<const BaseDeterminizationWithFluents*>(this) == NULL))
        return;

	cdf << "  (:functions" << std::endl;

	for (std::map<std::string, Function>::const_iterator If = domain_.functions().get_functions().begin() ; If != domain_.functions().get_functions().end() ; ++If)
	{
	    if (If->first == "total-time") // defined by default in PDDL2.1+
            continue;

		cdf << "    (" << (If->first);
		const TypeList& function_types = FunctionTable::parameters(If->second);

		unsigned int cnt = 1;

		if (domain_.requirements.typing)
		{
			for (TypeList::const_iterator It = function_types.begin() ; It != function_types.end() ; ++It)
			{
				cdf << " ?v" << cnt << " - " << (*It);
				cnt++;
			}

			//cdf << ") - " << TypeTable::NUMBER_NAME << std::endl;
			cdf << ")" << std::endl; // type "number" seems to be not accepted by the MFF parser
		}
		else
		{
			for (TypeList::const_iterator It = function_types.begin() ; It != function_types.end() ; ++It)
			{
				cdf << " ?v" << cnt;
				cnt++;
			}

			cdf << ")" << std::endl;
		}
	}

	cdf << "  )" << std::endl;
}


void BaseDeterminization::write_actions(std::ofstream& cdf) const
{
	for (ActionSchemaMap::const_iterator Ia = deterministic_action_schemas_.begin() ; Ia != deterministic_action_schemas_.end() ; ++Ia)
	{
		cdf << "  (:action " << (Ia->first) << std::endl;
		cdf << "    :parameters (";

		for (VariableList::const_iterator Iv = Ia->second->parameters().begin() ; Iv != Ia->second->parameters().end() ; ++Iv)
		{
			cdf << " ";
			Term(*Iv).print_with_type(cdf);
		}

//		if (domain_.requirements.typing)
//		{
//			for (VariableList::const_iterator Iv = Ia->second->parameters().begin() ; Iv != Ia->second->parameters().end() ; ++Iv)
//				cdf << " " << (*Iv) << " - " << TermTable::type(*Iv);
//		}
//		else
//		{
//			for (VariableList::const_iterator Iv = Ia->second->parameters().begin() ; Iv != Ia->second->parameters().end() ; ++Iv)
//				cdf << " " << (*Iv);
//		}

		cdf << ")" << std::endl; // end of parameters

		cdf << "    :precondition" << std::endl;
		cdf << (Ia->second->precondition()) << std::endl;

		if (Ia->second->effect().empty())
			cdf << "    :effect" << std::endl << "(and)" << std::endl;
		else
			cdf << "    :effect" << std::endl << (Ia->second->effect()) << std::endl;

		cdf << "  )" << std::endl;
	}
}


void BaseDeterminization::make_problem(const AtomSet& init_atoms, const ValueMap& init_values, const StateFormula& goal_states, const Expression* optimization_expression)
{
	char dirpath[dirpath_length_];
	getcwd(dirpath, dirpath_length_);
	chdir ("/tmp/");
	std::ofstream cdf(problem_file_name_.c_str(), std::ios_base::out | std::ios_base::trunc);

	cdf << "(define (problem ppddl_planner_deterministic_" << problem_.name() << ")" << std::endl;
	cdf << "  (:domain ppddl_planner_deterministic_" << domain_.name() << ")" << std::endl;
	write_requirements(cdf);
	write_objects(cdf);
	write_init_formula(cdf, init_atoms, init_values);
	write_goal_formula(cdf, goal_states);

	if (optimization_expression)
        write_optimization_formula(cdf, *optimization_expression);

	cdf << ")" << std::endl;

	cdf.close();
	chdir(dirpath);
}


void BaseDeterminization::write_objects(std::ofstream& cdf) const
{
	cdf << "  (:objects";

	if (domain_.requirements.typing)
	{
		for (std::map<std::string, Object>::const_iterator Iob = problem_.terms().get_objects().begin() ; Iob != problem_.terms().get_objects().end() ; ++Iob)
			cdf << " " << (Iob->first) << " - " << TermTable::type(Iob->second);
	}
	else
	{
		for (std::map<std::string, Object>::const_iterator Iob = problem_.terms().get_objects().begin() ; Iob != problem_.terms().get_objects().end() ; ++Iob)
			cdf << " " << (Iob->first);
	}

	cdf << ")" << std::endl;
}


void BaseDeterminization::write_init_formula(std::ofstream& cdf, const AtomSet& init_atoms, const ValueMap& init_values) const
{
	cdf << "  (:init";

	for (AtomSet::const_iterator Ia = problem_.init_atoms().begin() ; Ia != problem_.init_atoms().end() ; ++Ia)
	{
		if (PredicateTable::static_predicate((*Ia)->predicate()))
			cdf << " " << (**Ia);
	}

	for (AtomSet::const_iterator Ia = init_atoms.begin() ; Ia != init_atoms.end() ; ++Ia)
		cdf << " " << (**Ia);

	if (!domain_.functions().get_functions().empty())
	{
		for (ValueMap::const_iterator Iv = problem_.init_values().begin() ; Iv != problem_.init_values().end() ; ++Iv)
		{
			if (FunctionTable::static_function(Iv->first->function()))
				cdf << " (= " << (*(Iv->first)) << " " << (Iv->second.double_value()) << ")";
		}

		for (ValueMap::const_iterator Iv = init_values.begin() ; Iv != init_values.end() ; ++Iv)
			cdf << " (= " << (*(Iv->first)) << " " << (Iv->second.double_value()) << ")";
	}

	cdf << "  )" << std::endl;
}


void BaseDeterminization::write_goal_formula(std::ofstream& cdf, const StateFormula& goal_states) const
{
	cdf << "  (:goal" << std::endl << goal_states << std::endl << "  )" << std::endl;
}


void BaseDeterminization::write_optimization_formula(std::ofstream& cdf, const Expression& optimization_expression) const
{
	cdf << "  (:metric maximize" << std::endl << optimization_expression << std::endl << "  )" << std::endl;
}


const Action* BaseDeterminization::action(const std::list<std::string>& action_tokens) const
{
	if (!action_tokens.empty())
	{
		// Finds the objects
		std::list<const Object*> action_objects;

		for (std::list<std::string>::const_iterator Ip = ++(action_tokens.begin()) ; Ip != action_tokens.end() ; ++Ip)
		{
			const Object* param = problem_.terms().find_object(*Ip);

			if (param)
				action_objects.push_back(param);
			else
				throw BaseException(std::string("unknown parameter ") + (*Ip), "BaseDeterminization::action");
		}

		// Finds the instantiated action

		for (ActionSet::const_iterator Ia = deterministic_actions_.begin() ; Ia != deterministic_actions_.end() ; ++Ia)
		{
			if (((*Ia)->name()) == action_tokens.front())
			{
				if (((*Ia)->arguments().size()) != action_objects.size())
					continue;

				std::list<const Object*>::const_iterator Ip1 = action_objects.begin();

				for (ObjectList::const_iterator Ip2 = (*Ia)->arguments().begin() ; Ip2 != (*Ia)->arguments().end() ; ++Ip2)
				{
					if ((**Ip1) != (*Ip2))
						break;

					++Ip1;
				}

				if (Ip1 != action_objects.end())
					continue;
				else
					return (*Ia);
			}
		}
	}

	// This line should never be reached
	throw BaseException("empty action", "BaseDeterminization::action");
}


void BaseDeterminization::effect(const TransitionsBlackbox& tb, const PddlState& income, const Action& deterministic_action, PddlState& outcome, const Action*& probabilistic_action) const
{
	AtomList _adds, _deletes;
	UpdateList _updates;

	if (!(deterministic_action.precondition().holds(problem_.terms(), income.atoms(), income.values())))
	{
		std::ostringstream error_str;
		error_str << "action " << deterministic_action << " is not applicable in state " << income;
		throw BaseException(error_str.str(), "BaseDeterminization::effect");
	}

	probabilistic_action = &deterministic_to_probabilistic_action(deterministic_action);
	deterministic_action.effect().state_change(_adds, _deletes, _updates, problem_.terms(), income.atoms(), income.values());
	AtomSet adds;
	std::copy(_adds.begin(), _adds.end(), std::inserter(adds, adds.begin()));
	AtomSet deletes;
	std::copy(_deletes.begin(), _deletes.end(), std::inserter(deletes, deletes.begin()));
	UpdateSet updates;
	std::copy(_updates.begin(), _updates.end(), std::inserter(updates, updates.begin()));

	AtomSet temp_atoms;
	std::set_difference(income.atoms().begin(), income.atoms().end(),
			deletes.begin(), deletes.end(),
			std::inserter(temp_atoms, temp_atoms.begin()));
	std::set_union(temp_atoms.begin(), temp_atoms.end(),
			adds.begin(), adds.end(),
			std::inserter(outcome._atoms_(), outcome._atoms_().begin()));

	outcome.copy_values(income);
	add_artificial_values(tb, outcome);

	for (UpdateSet::const_iterator Iup = updates.begin() ; Iup != updates.end() ; ++Iup)
		(*Iup)->affect(outcome._values_());

    remove_artificial_values(tb, outcome);
}


const Action& BaseDeterminization::deterministic_to_probabilistic_action(const Action& deterministic_action) const
{
	std::string probabilistic_action_name = rename_action(deterministic_action.name());

	for (ActionSet::const_iterator Ia = problem_.actions().begin() ; Ia != problem_.actions().end() ; ++Ia)
	{
		if (((*Ia)->name() == probabilistic_action_name) && ((*Ia)->arguments() == deterministic_action.arguments()))
			return **Ia;
	}

	std::ostringstream oss;
	oss << "no probabilistic action corresponding to the deterministic action " << deterministic_action;
	throw BaseException(oss.str(), "BaseDeterminization::deterministic_to_probabilistic_action");
}


void BaseDeterminization::add_artificial_values(const TransitionsBlackbox& tb, PddlState& outcome) const
{
    if (tb.reward_fluent())
		outcome._values_().insert(std::make_pair(tb.reward_fluent(), Rational(0))).first;

	if (tb.total_time_fluent())
		outcome._values_().insert(std::make_pair(tb.total_time_fluent(), Rational(0))).first;

	if (tb.goal_achieved_fluent())
		outcome._values_().insert(std::make_pair(tb.goal_achieved_fluent(), Rational(0))).first;
}


void BaseDeterminization::remove_artificial_values(const TransitionsBlackbox& tb, PddlState& outcome) const
{
    if (tb.reward_fluent())
		outcome._values_().erase(tb.reward_fluent());

	if (tb.total_time_fluent())
		outcome._values_().erase(tb.total_time_fluent());

	if (tb.goal_achieved_fluent())
		outcome._values_().erase(tb.goal_achieved_fluent());
}


// FLUENTS CASE

BaseDeterminizationWithFluents::BaseDeterminizationWithFluents(const Domain& dm, const Problem& pb, unsigned int plan_length)
try : BaseDeterminization(dm, pb), plan_length_(plan_length)
{
    // We must add the artificial fluents planner-probability and planner-step-counter to the original probabilistic domain too...
    probability_fluent_ = &Fluent::make(const_cast<Domain&>(dm).functions().add_function("planner-probability"), TermList());
    RCObject::ref(probability_fluent_);
    step_counter_fluent_ = &Fluent::make(const_cast<Domain&>(dm).functions().add_function("planner-step-counter"), TermList());
    RCObject::ref(step_counter_fluent_);

    reward_fluent_ = Fluent::fluent(*(pb.domain().functions().find_function("reward")));
    if (!reward_fluent_)
        throw BaseException("No reward fluent defined in this domain");

    for (ValueMap::const_iterator iv = pb.init_values().begin() ; iv != pb.init_values().end() ; ++iv)
    {
        if (FunctionTable::static_function(iv->first->function()))
        {
            static_fluents_values_.insert(std::make_pair(iv->first, iv->second));
        }
    }

    probability_min_ = 1.0;
    probability_max_ = 0.0;
    reward_min_ = std::numeric_limits<double>::max();
    reward_max_ = - std::numeric_limits<double>::max();

    Rational::print_double(true);
}
catch (BaseException& error)
{
	error.push_function_backtrace("BaseDeterminizationWithFluents::BaseDeterminizationWithFluents");
	throw;
}


BaseDeterminizationWithFluents::~BaseDeterminizationWithFluents()
{
    RCObject::destructive_deref(probability_fluent_);
    RCObject::destructive_deref(step_counter_fluent_);
}


void BaseDeterminizationWithFluents::make_problem(const AtomSet& init_atoms, const ValueMap& init_values, const StateFormula& goal_states, const Expression* optimization_expression)
{
    ValueMap iv = init_values;
    iv.insert(std::make_pair(reward_fluent_, 0));
    iv.insert(std::make_pair(probability_fluent_, 0));
    iv.insert(std::make_pair(step_counter_fluent_, 0));

    const Expression& vexp = *new Value((int) plan_length_);
    RCObject::ref(&vexp);
    const StateFormula& gtfml = GreaterThanOrEqualTo::make(*step_counter_fluent_, vexp);
    RCObject::ref(&gtfml);
    RCObject::destructive_deref(&vexp);
    const StateFormula& cfml = goal_states && gtfml;
    RCObject::ref(&cfml);
    RCObject::destructive_deref(&gtfml);

    BaseDeterminization::make_problem(init_atoms, iv, cfml, optimization_expression);

    RCObject::destructive_deref(&cfml);
}


void BaseDeterminizationWithFluents::make_problem(const AtomSet& init_atoms, const ValueMap& init_values, const Expression& optimization_expression)
{
    ValueMap iv = init_values;
    iv.insert(std::make_pair(reward_fluent_, 0));
    iv.insert(std::make_pair(probability_fluent_, 0));
    iv.insert(std::make_pair(step_counter_fluent_, 0));

    const Expression& vexp = *new Value((int) plan_length_);
    RCObject::ref(&vexp);
    const StateFormula& gtfml = GreaterThanOrEqualTo::make(*step_counter_fluent_, vexp);
    RCObject::ref(&gtfml);
    RCObject::destructive_deref(&vexp);

    BaseDeterminization::make_problem(init_atoms, iv, gtfml, &optimization_expression);

    RCObject::destructive_deref(&gtfml);
}


void BaseDeterminizationWithFluents::add_artificial_values(const TransitionsBlackbox& tb, PddlState& outcome) const
{
    outcome._values_().insert(std::make_pair(reward_fluent_, 0));
    outcome._values_().insert(std::make_pair(probability_fluent_, 0));
    outcome._values_().insert(std::make_pair(step_counter_fluent_, 0));
}


void BaseDeterminizationWithFluents::remove_artificial_values(const TransitionsBlackbox& tb, PddlState& outcome) const
{
    outcome._values_().erase(reward_fluent_);
    outcome._values_().erase(probability_fluent_);
    outcome._values_().erase(step_counter_fluent_);
}
