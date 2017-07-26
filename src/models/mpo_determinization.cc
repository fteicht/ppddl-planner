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

#include <cmath>
#include <fstream>
#include <sstream>
#include <typeinfo>
#include <mdpsim/domains.h>
#include <mdpsim/problems.h>

#include "mpo_determinization.h"

// GOAL CASE

void MostProbableOutcomeDeterminizationGoal::generate_deterministic_actions()
{
	for (ActionSchemaMap::const_iterator Ia = domain_.actions().begin() ; Ia != domain_.actions().end() ; ++Ia)
	{
		//const Effect* eff = generate_deterministic_effect(Ia->second->effect());
		const Effect* eff = Ia->second->effect().mpo_determinization();
		add_deterministic_action_schema(*(Ia->second), Ia->first, *eff);
		RCObject::destructive_deref(eff);
	}
}


const Effect* MostProbableOutcomeDeterminizationGoal::generate_deterministic_effect(const Effect& probabilistic_effect) const
{
	if (probabilistic_effect.empty())
	{
		RCObject::ref(&Effect::EMPTY);
		return &Effect::EMPTY;
	}

	const ConjunctiveEffect* conjunctive_effect = dynamic_cast<const ConjunctiveEffect*>(&probabilistic_effect);

	if (conjunctive_effect)
	{
		const Effect* eff = &Effect::EMPTY;
		RCObject::ref(eff);

		for (EffectList::const_iterator Ieff = conjunctive_effect->conjuncts().begin() ; Ieff != conjunctive_effect->conjuncts().end() ; ++Ieff)
		{
			const Effect* effect_temp = generate_deterministic_effect(**Ieff);
			const Effect* efftmp = eff;
			eff = &(*efftmp && (*effect_temp));
			RCObject::ref(eff);
			RCObject::destructive_deref(efftmp);
			RCObject::destructive_deref(effect_temp);
		}

		return eff;
	}

	const ConditionalEffect* conditional_effect = dynamic_cast<const ConditionalEffect*>(&probabilistic_effect);

	if (conditional_effect)
	{
		const Effect* efftmp = generate_deterministic_effect(conditional_effect->effect());
		const Effect* deterministic_effect = &ConditionalEffect::make(conditional_effect->condition(), *efftmp);
		RCObject::ref(deterministic_effect);
		RCObject::destructive_deref(efftmp);
		return deterministic_effect;
	}

	const ProbabilisticEffect* prob_effect = dynamic_cast<const ProbabilisticEffect*>(&probabilistic_effect);

	if (prob_effect)
	{
		double max_prob = 0.0;
		unsigned int max_outcome = 0;
		double prob_sum = 0.0;

		for (unsigned int cnt = 0 ; cnt < (prob_effect->size()) ; cnt++)
		{
			double prob = prob_effect->probability(cnt).double_value();
			prob_sum += prob;

			if (prob > max_prob)
			{
				max_prob = prob;
				max_outcome = cnt;
			}
		}

		if ((1.0 - prob_sum) > max_prob)
		{
			RCObject::ref(&Effect::EMPTY);
			return &Effect::EMPTY;
		}
		else
			return generate_deterministic_effect(prob_effect->effect(max_outcome));
	}

	const QuantifiedEffect* quantified_effect = dynamic_cast<const QuantifiedEffect*>(&probabilistic_effect);

	if (quantified_effect)
	{
		const Effect* efftmp = generate_deterministic_effect(quantified_effect->effect());
		const Effect* deterministic_effect = &QuantifiedEffect::make(quantified_effect->parameters(), *efftmp);
		RCObject::ref(deterministic_effect);
		RCObject::destructive_deref(efftmp);
		return deterministic_effect;
	}

	const UpdateEffect* update_effect = dynamic_cast<const UpdateEffect*>(&probabilistic_effect);

	if (update_effect)
	{
		RCObject::ref(&Effect::EMPTY);
		return &Effect::EMPTY; // no fluents allowed in IPPC'08
	}

	// All other cases : copy the probabilistic effect
	RCObject::ref(&probabilistic_effect);
	return &probabilistic_effect;
}


// REWARD CASE

void MostProbableOutcomeDeterminizationReward::generate_deterministic_actions()
{
	for (ActionSchemaMap::const_iterator Ia = domain_.actions().begin() ; Ia != domain_.actions().end() ; ++Ia)
	{
		//const Effect* eff = generate_deterministic_effect(Ia->second->effect());
		double probability, reward;
		const Effect* eff = Ia->second->effect().mpo_fdeterminization(probability, reward, static_fluents_values_);
		const Expression& sexp = *new Value(1);
		RCObject::ref(&sexp);
		const Effect& seff = UpdateEffect::make(*new Increase(*step_counter_fluent_, sexp));
		RCObject::ref(&seff);
		RCObject::destructive_deref(&sexp);
		const Effect& efftmp = (*eff) && seff;
		RCObject::ref(&efftmp);
		RCObject::destructive_deref(eff);
		RCObject::destructive_deref(&seff);

		if (probability != 1.0d)
        {
            const Expression& vexp = *new Value(std::log(probability));
            RCObject::ref(&vexp);
            const Effect* ueff = &UpdateEffect::make(*new Increase(*probability_fluent_, vexp));
            RCObject::ref(ueff);
            RCObject::destructive_deref(&vexp);
            const Effect& aeff = efftmp && (*ueff);
            RCObject::ref(&aeff);
            RCObject::destructive_deref(&efftmp);
            RCObject::destructive_deref(ueff);
            add_deterministic_action_schema(*(Ia->second), Ia->first, aeff);
            RCObject::destructive_deref(&aeff);
        }
        else
        {
            add_deterministic_action_schema(*(Ia->second), Ia->first, efftmp);
            RCObject::destructive_deref(&efftmp);
        }

		probability_min_ = std::min(probability_min_, probability);
		probability_max_ = std::max(probability_max_, probability);
		reward_min_ = std::min(reward_min_, reward);
		reward_max_ = std::max(reward_max_, reward);
	}
}


const Effect* MostProbableOutcomeDeterminizationReward::generate_deterministic_effect(const Effect& probabilistic_effect) const
{
	// Deprecated
}
