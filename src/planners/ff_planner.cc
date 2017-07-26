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

extern "C" {
#include <unistd.h>
}

#include <cstdio>
#include <cstring>
#include <string>
#include <algorithm>
#include <set>
#include <mdpsim/problems.h>

#include "ff_planner.h"
#include "models/base_determinization.h"
#include "commons/base_exceptions.h"
#include "config.h"

void FFPlanner::read_plan(FILE* ff_stdout, plan_t& plan)
{
	char stdout_line[stdout_line_length_];
	extern int verbosity; // declared in main.cc

	// Searches for "ff: found legal plan as follows"
	bool plan_found = false;

	while (fgets(stdout_line, stdout_line_length_, ff_stdout))
	{
		if (verbosity >= 2)
			printf(stdout_line);

		if (strstr(stdout_line, "ff: found legal plan as follows"))
		{
			plan_found = true;
			break;
		}
	}

	if (!plan_found)
		return;

	unsigned int action_nb = 0;
	char action_nb_str[action_nb_string_length_];
	sprintf(action_nb_str, "%u: ", action_nb);
	char* action_str;

	while (fgets(stdout_line, stdout_line_length_, ff_stdout))
	{
		if (verbosity >= 2)
			printf(stdout_line);

		if ((action_str = strstr(stdout_line, action_nb_str)))
		{
			push_action(plan, action_str);
			action_nb++;
			sprintf(action_nb_str, "%u: ", action_nb);
		}
	}
}


void FFPlanner::push_action(plan_t& plan, const char* action_str)
{
	// Split string into tokens

	std::list<std::string> action_tokens;
	const char* current_pos = strchr(action_str, ' ') + 1;
	size_t token_length;

	while ((token_length = strcspn(current_pos, " \n")) > 0)
	{
		std::string action_name(current_pos, token_length);

		if (action_name == std::string("REACH-GOAL")) // REACH-GOAL is an artificial FF action
			return;

		action_tokens.push_back(action_name);
		std::transform(action_tokens.back().begin(), action_tokens.back().end(), action_tokens.back().begin(), (int(*)(int)) std::tolower);
		current_pos += token_length + 1;
	}

	plan.push_back(det_.action(action_tokens));
}


void SFFPlanner::execute_planner(bool optimization_mode) const
{

    std::string arg1 = std::string("/tmp/") + (det_.get_domain_file_name());
    std::string arg2 = std::string("/tmp/") + (det_.get_problem_file_name());
    execlp(FF_COMMAND, "ff", "-o", arg1.c_str(), "-f", arg2.c_str(), NULL);
}


std::string SFFPlanner::planner_command(bool optimization_mode) const
{
    std::string ff_command(FF_COMMAND);
	ff_command += " -o /tmp/" + (det_.get_domain_file_name()) + " -f /tmp/" + (det_.get_problem_file_name());
	return ff_command;
}


void NFFPlanner::execute_planner(bool optimization_mode) const
{
    std::string arg1 = std::string("/tmp/") + (det_.get_domain_file_name());
    std::string arg2 = std::string("/tmp/") + (det_.get_problem_file_name());

    if (optimization_mode)
        execlp(METRIC_FF_COMMAND, "ff", "-o", arg1.c_str(), "-f", arg2.c_str(), "-O", NULL);
    else
        execlp(METRIC_FF_COMMAND, "ff", "-o", arg1.c_str(), "-f", arg2.c_str(), NULL);
}


std::string NFFPlanner::planner_command(bool optimization_mode) const
{
    std::string mff_command(METRIC_FF_COMMAND);

    if (optimization_mode)
        mff_command += " -o /tmp/" + (det_.get_domain_file_name()) + " -f /tmp/" + (det_.get_problem_file_name()) + " -O";
    else
        mff_command += " -o /tmp/" + (det_.get_domain_file_name()) + " -f /tmp/" + (det_.get_problem_file_name());

	return mff_command;
}



