/*
   ppddl-planner - client for IPPC'08

   Copyright (C) 2011 Florent Teichteil-Koenigsbuch and Guillaume Infantes and Ugur Kuter

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

#include "determinization_heuristic.h"
#include "models/mpo_determinization.h"
#include "models/ao_determinization.h"
#include "planners/ff_planner.h"
#include "commons/base_exceptions.h"


DeterminizationHeuristic::DeterminizationHeuristic(const Problem& pb, double discount_factor, unsigned int plan_length, determinization_t determinization_type, deterministic_planner_t deterministic_planner_type)
try : BaseHeuristic(discount_factor), determinization_type_(determinization_type), deterministic_planner_type_(deterministic_planner_type), problem_(pb)
{
	switch (determinization_type)
	{
	case DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL :
		determinization_ = new MostProbableOutcomeDeterminizationGoal(pb.domain(), pb);
		break;

    case DETERMINIZATION_MOST_PROBABLE_OUTCOME_REWARD :
		determinization_ = new MostProbableOutcomeDeterminizationReward(pb.domain(), pb, plan_length);
		break;

	case DETERMINIZATION_ALL_OUTCOMES_GOAL :
		determinization_ = new AllOutcomesDeterminizationGoal(pb.domain(), pb);
		break;

    case DETERMINIZATION_ALL_OUTCOMES_REWARD :
		determinization_ = new AllOutcomesDeterminizationReward(pb.domain(), pb, plan_length);
		break;
	}

	switch (deterministic_planner_type)
	{
	case DETERMINISTIC_PLANNER_FF :
		deterministic_planner_ = new SFFPlanner(*determinization_);
		break;

    case DETERMINISTIC_PLANNER_MFF :
		deterministic_planner_ = new NFFPlanner(*determinization_);
		break;
	}

	determinization_->make_domain();
}
catch (BaseException& error)
{
	error.push_function_backtrace("DeterminizationHeuristic::DeterminizationHeuristic");
	throw;
}


DeterminizationHeuristic::~DeterminizationHeuristic()
{
	delete deterministic_planner_;
	delete determinization_;
}
