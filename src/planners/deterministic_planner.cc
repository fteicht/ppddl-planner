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

#include <cstdlib>
#include <cstdio>
#include <csignal>
extern "C" {
#include <unistd.h>
}

#include <iostream>

#define	LAUNCH_PLANNER_READ     0
#define	LAUNCH_PLANNER_WRITE    1

#include "deterministic_planner.h"
#include "commons/base_exceptions.h"

void DeterministicPlanner::solve(plan_t& plan, bool optimization_mode)
{
	FILE * planner_stdout = popen(planner_command(optimization_mode).c_str(), "r");

	if (!planner_stdout)
	{
		std::cerr << "WARNING: unable to run the deterministic planner because popen fails in 'DeterministicPlanner::solve'" << std::endl;
		throw BaseException("unable to call the deterministic planner", "DeterministicPlanner::solve");
	}

	read_plan(planner_stdout, plan);

	if (pclose(planner_stdout) == -1)
		std::cerr << "WARNING: unable to close the deterministic planner pipe in 'DeterministicPlanner::solve'" << std::endl;
}


void DeterministicPlanner::solve_with_timeout(plan_t& plan, unsigned int timeout, bool optimization_mode)
{
	FILE* planner_stdout = launch_planner(timeout, optimization_mode);

	if (!planner_stdout)
	{
		std::cerr << "WARNING: unable to run the deterministic planner because popen fails in 'DeterministicPlanner::solve_with_timeout'" << std::endl;
		throw BaseException("unable to call the deterministic planner", "DeterministicPlanner::solve_with_timeout");
	}

	read_plan(planner_stdout, plan);

	if (pclose(planner_stdout) == -1)
		std::cerr << "WARNING: unable to close the deterministic planner pipe in 'DeterministicPlanner::solve_with_timeout'" << std::endl;
}


FILE* DeterministicPlanner::launch_planner(unsigned int time_limit, bool optimization_mode) const
{
    int	pfp[2], pid;		/* the pipe and the process	*/
	int	parent_end, child_end;	/* of pipe 			*/

    parent_end = LAUNCH_PLANNER_READ;
    child_end = LAUNCH_PLANNER_WRITE;

	if ( pipe(pfp) == -1 ) 			/* get a pipe		*/
		return NULL;
	if ( (pid = fork()) == -1 ){		/* and a process	*/
		close(pfp[0]);			/* or dispose of pipe	*/
		close(pfp[1]);
		return NULL;
	}

	/* --------------- parent code here ------------------- */
	/*   need to close one end and fdopen other end		*/

	if ( pid > 0 ){
        signal(SIGCHLD, SIG_IGN);
		if (close( pfp[child_end] ) == -1 )
			return NULL;
		return fdopen( pfp[parent_end] , "r");	/* same mode */
	}

	/* --------------- child code here --------------------- */
	/*   need to redirect stdin or stdout then exec the cmd	 */

    if ( close(pfp[parent_end]) == -1 )	/* close the other end	*/
		_exit(1);			/* do NOT return	*/

	if ( dup2(pfp[child_end], child_end) == -1 )
		_exit(1);

	if ( close(pfp[child_end]) == -1 )	/* done with this one	*/
		_exit(1);

	/* all set to run cmd	*/
    alarm(time_limit);
    execute_planner(optimization_mode);
	_exit(1);
}
