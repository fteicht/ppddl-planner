/*
   ppddl-planner - client for IPPC'08

   Copyright (C) 2009 Florent Teichteil-Koenigsbuch and Guillaume Infantes and Ugur Kuter

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

#include "ao_determinization.h"

// GOAL CASE

void AllOutcomesDeterminizationGoal::generate_deterministic_actions()
{
	for (ActionSchemaMap::const_iterator Ia = domain_.actions().begin() ; Ia != domain_.actions().end() ; ++Ia)
	{
		EffectList eff;
		//generate_deterministic_effects(Ia->second->effect(), eff);
		Ia->second->effect().ao_determinization(eff);
		unsigned int cnt = 0;

		for (EffectList::const_iterator Ie = eff.begin() ; Ie != eff.end() ; ++Ie)
		{
			std::ostringstream oss;
			oss << (Ia->first) << "_det_" << cnt;
			add_deterministic_action_schema(*(Ia->second), oss.str(), **Ie);
			RCObject::destructive_deref(*Ie);
			cnt++;
		}
	}
}


void AllOutcomesDeterminizationGoal::generate_deterministic_effects(const Effect& probabilistic_effect, EffectList& deterministic_effects) const
{
	if (probabilistic_effect.empty())
	{
		deterministic_effects.push_back(&Effect::EMPTY);
		RCObject::ref(&Effect::EMPTY);
		return;
	}

	const ConjunctiveEffect* conjunctive_effect = dynamic_cast<const ConjunctiveEffect*>(&probabilistic_effect);

	if (conjunctive_effect)
	{
		deterministic_effects.push_back(&Effect::EMPTY);
		RCObject::ref(&Effect::EMPTY);

		for (EffectList::const_iterator Ieff = conjunctive_effect->conjuncts().begin() ; Ieff != conjunctive_effect->conjuncts().end() ; ++Ieff)
		{
			EffectList effect_temp;
			generate_deterministic_effects(**Ieff, effect_temp);
			EffectList efftmp = deterministic_effects;
			deterministic_effects.clear();

			for (EffectList::const_iterator Ieffbis = effect_temp.begin() ; Ieffbis != effect_temp.end() ; ++Ieffbis)
			{
				for (EffectList::const_iterator Iefftmp = efftmp.begin() ; Iefftmp != efftmp.end() ; ++Iefftmp)
				{
					const Effect* eff = &((**Iefftmp) && (**Ieffbis));
					RCObject::ref(eff);
					deterministic_effects.push_back(eff);
				}

				RCObject::destructive_deref(*Ieffbis);
			}

			for (EffectList::const_iterator Iefftmp = efftmp.begin() ; Iefftmp != efftmp.end() ; ++Iefftmp)
				RCObject::destructive_deref(*Iefftmp);
		}

		return;
	}

	const ConditionalEffect* conditional_effect = dynamic_cast<const ConditionalEffect*>(&probabilistic_effect);

	if (conditional_effect)
	{
		generate_deterministic_effects(conditional_effect->effect(), deterministic_effects);

		for (EffectList::iterator Ieff = deterministic_effects.begin() ; Ieff != deterministic_effects.end() ; ++Ieff)
		{
			const Effect* efftmp = *Ieff;
			*Ieff = &ConditionalEffect::make(conditional_effect->condition(), *efftmp);
			RCObject::ref(*Ieff);
			RCObject::destructive_deref(efftmp);
		}

		return;
	}

	const ProbabilisticEffect* prob_effect = dynamic_cast<const ProbabilisticEffect*>(&probabilistic_effect);

	if (prob_effect)
	{
		double prob = 0.0;

		for (unsigned int cnt = 0 ; cnt < (prob_effect->size()) ; cnt++)
		{
			EffectList effect_temp;
			generate_deterministic_effects(prob_effect->effect(cnt), effect_temp);
			deterministic_effects.insert(deterministic_effects.end(), effect_temp.begin(), effect_temp.end());
			prob += prob_effect->probability(cnt).double_value();
		}

		if (prob < 1.0)
		{
			deterministic_effects.push_back(&Effect::EMPTY);
			RCObject::ref(&Effect::EMPTY);
		}

		return;
	}

	const QuantifiedEffect* quantified_effect = dynamic_cast<const QuantifiedEffect*>(&probabilistic_effect);

	if (quantified_effect)
	{
		generate_deterministic_effects(quantified_effect->effect(), deterministic_effects);

		for (EffectList::iterator Ieff = deterministic_effects.begin() ; Ieff != deterministic_effects.end() ; ++Ieff)
		{
			const Effect* efftmp = *Ieff;
			*Ieff = &QuantifiedEffect::make(quantified_effect->parameters(), *efftmp);
			RCObject::ref(*Ieff);
			RCObject::destructive_deref(efftmp);
		}

		return;
	}

	const UpdateEffect* update_effect = dynamic_cast<const UpdateEffect*>(&probabilistic_effect);

	if (update_effect)
	{
		deterministic_effects.push_back(&Effect::EMPTY);
		RCObject::ref(&Effect::EMPTY);
		return; // no fluents allowed in IPPC'08
	}

	// All other cases : copy the probabilistic effect
	deterministic_effects.push_back(&probabilistic_effect);
	RCObject::ref(&probabilistic_effect);
}


std::string AllOutcomesDeterminizationGoal::rename_action(const std::string& deterministic_name) const
{
	return deterministic_name.substr(0, deterministic_name.rfind("_det_"));
}


// REWARD CASE

void AllOutcomesDeterminizationReward::generate_deterministic_actions()
{
	for (ActionSchemaMap::const_iterator Ia = domain_.actions().begin() ; Ia != domain_.actions().end() ; ++Ia)
	{
		EffectList eff;
		//generate_deterministic_effects(Ia->second->effect(), eff);
		std::vector<std::pair<double, double> > pr;
		Ia->second->effect().ao_fdeterminization(eff, pr, static_fluents_values_);
		unsigned int cnt = 0;

        std::vector<std::pair<double, double> >::const_iterator ipr = pr.begin();
		for (EffectList::const_iterator Ie = eff.begin() ; Ie != eff.end() ; ++Ie)
		{
			std::ostringstream oss;
			oss << (Ia->first) << "_det_" << cnt;
			const Expression& sexp = *new Value(1);
            RCObject::ref(&sexp);
            const Effect& seff = UpdateEffect::make(*new Increase(*step_counter_fluent_, sexp));
            RCObject::ref(&seff);
            RCObject::destructive_deref(&sexp);
			const Effect& efftmp = (**Ie) && seff;
			RCObject::ref(&efftmp);
			RCObject::destructive_deref(*Ie);
			RCObject::destructive_deref(&seff);

			if (ipr->first != 1.0d)
            {
                const Expression& vexp = *new Value(std::log(ipr->first));
                RCObject::ref(&vexp);
                const Effect* ueff = &UpdateEffect::make(*new Increase(*probability_fluent_, vexp));
                RCObject::ref(ueff);
                RCObject::destructive_deref(&vexp);
                const Effect& aeff = efftmp && (*ueff);
                RCObject::ref(&aeff);
                RCObject::destructive_deref(&efftmp);
                RCObject::destructive_deref(ueff);
                add_deterministic_action_schema(*(Ia->second), oss.str(), aeff);
                RCObject::destructive_deref(&aeff);
            }
            else
            {
                add_deterministic_action_schema(*(Ia->second), oss.str(), efftmp);
                RCObject::destructive_deref(&efftmp);
            }

			probability_min_ = std::min(probability_min_, ipr->first);
            probability_max_ = std::max(probability_max_, ipr->first);
            reward_min_ = std::min(reward_min_, ipr->second);
            reward_max_ = std::max(reward_max_, ipr->second);
			cnt++;
			++ipr;
		}
	}
}


void AllOutcomesDeterminizationReward::generate_deterministic_effects(const Effect& probabilistic_effect, EffectList& deterministic_effects) const
{
	// deprecated
}


std::string AllOutcomesDeterminizationReward::rename_action(const std::string& deterministic_name) const
{
	return deterministic_name.substr(0, deterministic_name.rfind("_det_"));
}

