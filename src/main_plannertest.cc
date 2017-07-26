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

#include <config.h>
#include "client.h"
#include "states.h"
#include "problems.h"
#include "domains.h"
#include "actions.h"
#include <iostream>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#if HAVE_GETOPT_LONG
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <getopt.h>
#else
#include "port/getopt.h"
#endif

#include "planners/test_planner.h"
#include "algorithms/heuristic_algorithm.h"
#include "commons/base_exceptions.h"


/* The parse function. */
extern int yyparse();
/* File to parse. */
extern FILE* yyin;
/* Name of current file. */
std::string current_file;
/* Level of warnings. */
int warning_level;
/* Verbosity level. */
int verbosity;
/* Current problem hack */
const Problem* current_problem_hack_;

static int gvi_flag = 0;
static int svi_flag = 0;
static int gfsp_flag = 0;
static int sfsp_flag = 0;
static int gfsps_flag = 0;
static int sfsps_flag = 0;
static int gbfdt_flag = 0;
static int sbfdt_flag = 0;
static int gtrfsps_flag = 0;
static int gpsps_flag = 0;
static int gfdp_flag = 0;
static int glao_flag = 0;
static int slao_flag = 0;
static int gilao_flag = 0;
static int grtdp_flag = 0;
static int srtdp_flag = 0;
static int glrtdp_flag = 0;
static int slrtdp_flag = 0;
static int gldfs_flag = 0;
static int sldfs_flag = 0;
static int god_flag = 0;
static int grdpg_flag = 0;
static int grdrg_flag = 0;
static int grdbg_flag = 0;
static int grdrd_flag = 0;
static int grdro_flag = 0;
static int ggco_flag = 0;
static int gmvpfpi_flag = 0;
static int gmvhopi_flag = 0;
static int gmvpfas_flag = 0;
static int gmvhoas_flag = 0;
static int gmvpfssp_flag = 0;
static int sgco_flag = 0;

static int hzero_flag = 0;
static int hpstep_flag = 0;
static int hnstep_flag = 0;
static int hrstep_flag = 0;
static int hrrewards_flag = 0;
static int hrbounds_flag = 0; // relaxed planning graph heuristic
static int hebounds_flag = 0;
static int hmax_flag = 0;
static int hadd_flag = 0;
static int hdet_flag = 0;

static int dmpo_flag = 0;
static int dao_flag = 0;

static int dpff_flag = 0;
static int dpmff_flag = 0;

static int cgs_flag = 0;
static int gce_flag = 0;

/* Program options. */
static struct option long_options[] = {
  { "gvi", no_argument, &gvi_flag, 1 },
  { "svi", no_argument, &svi_flag, 1},
  { "gfsp", no_argument, &gfsp_flag, 1 },
  { "sfsp", no_argument, &sfsp_flag, 1 },
  { "gfsps", no_argument, &gfsps_flag, 1 },
  { "sfsps", no_argument, &sfsps_flag, 1 },
  { "gbfdt", no_argument, &gbfdt_flag, 1 },
  { "sbfdt", no_argument, &sbfdt_flag, 1 },
  { "gtrfsps", no_argument, &gtrfsps_flag, 1 },
  { "gpsps", no_argument, &gpsps_flag, 1 },
  { "gfdp", no_argument, &gfdp_flag, 1 },
  { "glao", no_argument, &glao_flag, 1 },
  { "slao", no_argument, &slao_flag, 1 },
  { "gilao", no_argument, &gilao_flag, 1 },
  { "grtdp", no_argument, &grtdp_flag, 1 },
  { "srtdp", no_argument, &srtdp_flag, 1 },
  { "glrtdp", no_argument, &glrtdp_flag, 1 },
  { "slrtdp", no_argument, &slrtdp_flag, 1 },
  { "gldfs", no_argument, &gldfs_flag, 1 },
  { "sldfs", no_argument, &sldfs_flag, 1 },
  { "god", no_argument, &god_flag, 1 },
  { "grdpg", no_argument, &grdpg_flag, 1 },
  { "grdrg", no_argument, &grdrg_flag, 1 },
  { "grdbg", no_argument, &grdbg_flag, 1 },
  { "grdrd", no_argument, &grdrd_flag, 1 },
  { "grdro", no_argument, &grdro_flag, 1 },
  { "ggco", no_argument, &ggco_flag, 1 },
  { "gmvpfpi", no_argument, &gmvpfpi_flag, 1},
  { "gmvhopi", no_argument, &gmvhopi_flag, 1},
  { "gmvpfas", no_argument, &gmvpfas_flag, 1},
  { "gmvhoas", no_argument, &gmvhoas_flag, 1},
  { "gmvpfssp", no_argument, &gmvpfssp_flag, 1},
  { "sgco", no_argument, &sgco_flag, 1},
  { "hzero", no_argument, &hzero_flag, 1 },
  { "hpstep", no_argument, &hpstep_flag, 1 },
  { "hnstep", no_argument, &hnstep_flag, 1 },
  { "hrstep", no_argument, &hrstep_flag, 1 },
  { "hrrewards", no_argument, &hrrewards_flag, 1 },
  { "hrbounds", no_argument, &hrbounds_flag, 1 },
  { "hebounds", no_argument, &hebounds_flag, 1 },
  { "hmax", no_argument, &hmax_flag, 1 },
  { "hadd", no_argument, &hadd_flag, 1 },
  { "hdet", no_argument, &hdet_flag, 1 },
  { "dmpo", no_argument, &dmpo_flag, 1 },
  { "dao", no_argument, &dao_flag, 1 },
  { "dpff", no_argument, &dpff_flag, 1 },
  { "dpmff", no_argument, &dpmff_flag, 1 },
  { "cgs", no_argument, &cgs_flag, 1},
  { "gce", no_argument, &gce_flag, 1},
  { "mode", required_argument, 0, 'M' },
  { "epsilon", optional_argument, 0, 'e' },
  { "discount-factor", optional_argument, 0, 'd' },
  { "probabilistic-threshold", optional_argument, 0, 'p' },
  { "number-of-particles", optional_argument, 0, 'N' },
  { "trajectories_max_depth", optional_argument, 0, 'T' },
  { "time-threshold", optional_argument, 0, 't' },
  { "trials-number", optional_argument, 0, 'n' },
  { "trials-max-depth", optional_argument, 0, 'm' },
  { "goals-limit", optional_argument, 0, 'l' },
  { "deterministic-probability-weight", optional_argument, 0, 'q' },
  { "deterministic-reward-weight", optional_argument, 0, 'r' },
  { "deterministic-planner-timeout", optional_argument, 0, 'x' },
  { "deterministic-planner-max-trials", optional_argument, 0, 'X' },
  { "proportion-updated-states", optional_argument, 0, 'u' },
  { "minimum-mean", optional_argument, 0, 'a' },
  { "maximum-variance", optional_argument, 0, 'b' },
  { "optimize-reward-determinization", no_argument, 0, 'O' },
  { "verbose", optional_argument, 0, 'v' },
  { "warnings", optional_argument, 0, 'W' },
  { "help", no_argument, 0, 'h' },
  { 0, 0, 0, 0 }
};
static const char OPTION_STRING[] = "OM:e:d:p:N:T:t:n:m:l:q:r:x:X:u:a:b:v::W::h";


/* Displays help. */
static void display_help() {
  std::cout << "usage: plannertest [options] [file ...]" << std::endl
            << "options:" << std::endl
            << "  --gvi\t"
            << "test VI graph version" << std::endl
            << "  --svi\t"
            << "test VI symbolic version" << std::endl
            << "  --gfsp\t"
            << "test FSP graph version" << std::endl
            << "  --sfsp\t"
            << "test FSP symbolic version" << std::endl
            << "  --gfsps\t"
            << "test FSP* graph version" << std::endl
            << "  --sfsps\t"
            << "test FSP* symbolic version" << std::endl
            << "  --gtrfsps\t"
            << "test TrFSP* graph version" << std::endl
			<< "  --gbfdt\t"
            << "test BFDT graph version" << std::endl
            << "  --sbfdt\t"
            << "test BFDT symbolic version" << std::endl
            << "  --gpsps\t"
            << "test PSP* graph version" << std::endl
            << "  --glao\t"
            << "test FDP graph version" << std::endl
            << "  --gfdp\t"
            << "test LAO* graph version" << std::endl
            << "  --slao\t"
            << "test LAO* symbolic version" << std::endl
            << "  --gilao\t"
            << "test Improved LAO* graph version" << std::endl
            << "  --grtdp\t"
            << "test RTDP graph version" << std::endl
            << "  --srtdp\t"
            << "test RTDP symbolic version" << std::endl
			<< "  --glrtdp\t"
            << "test LRTDP graph version" << std::endl
            << "  --slrtdp\t"
            << "test LRTDP symbolic version" << std::endl
			<< "  --gldfs\t"
            << "test LDFS graph version" << std::endl
            << "  --sldfs\t"
            << "test LDFS symbolic version" << std::endl
            << "  --god\t"
            << "test OD graph version" << std::endl
            << "  --grdpg\t"
            << "test RDPG graph version" << std::endl
            << "  --grdrg\t"
            << "test RDRG graph version" << std::endl
            << "  --grdbg\t"
            << "test RDBG graph version" << std::endl
            << "  --grdrd\t"
            << "test RDRD graph version" << std::endl
            << "  --grdro\t"
            << "test RDRO graph version" << std::endl
            << "  --ggco\t"
            << "test GCO graph version" << std::endl
			<< "  --sgco\t"
            << "test GCO symbolic version" << std::endl
            << "  --gmvpfpi\t"
            << "test MVPFPI graph version" << std::endl
			<< "  --gmvhopi\t"
            << "test MVHOPI graph version" << std::endl
		   << "  --gmvpfas\t"
            << "test MVPFAS graph version" << std::endl
			<< "  --gmvhoas\t"
            << "test MVHOAS graph version" << std::endl
			<< "  --gmvpfssp\t"
            << "test MVPFSSP graph version" << std::endl
            << "  --hzero\t"
            << "test zero heuristic" << std::endl
            << "  --hpstep\t"
            << "test PDDL step distance heuristic" << std::endl
            << "  --hnstep\t"
            << "test native step distance heuristic" << std::endl
            << "  --hrstep\t"
            << "test relaxed step distance heuristic" << std::endl
            << "  --hrrewards\t"
            << "test relaxed rewards heuristic" << std::endl
            << "  --hrbounds\t"
            << "test relaxed bounds (planning graph) heuristic" << std::endl
            << "  --hebounds\t"
            << "test exact bounds heuristic" << std::endl
			<< "  --hmax\t"
            << "test HMAX heuristic" << std::endl
			<< "  --hadd\t"
            << "test HADD heuristic" << std::endl
			<< "  --hdet\t"
            << "test HDET heuristic" << std::endl
            << "  --dmpo\t"
            << "test most probable outcome determinization" << std::endl
            << "  --dao\t"
            << "test all outcomes determinization" << std::endl
            << "  --dpff\t"
            << "test FF deterministic planner" << std::endl
            << "  --dpmff\t"
            << "test METRIC-FF deterministic planner" << std::endl
            << "  --cgs\t"
            << "compute exact goal-probability and goal-cost values" << std::endl
            << "  --gce\t"
            << "use the GCE algorithm for computing goal-probability and goal-cost values" << std::endl
            << "  -M m, --mode=m\t"
            << "test planner mode (complete or anytime)" << std::endl
  			<< "  -e e, --epsilon=e\t"
  			<< "epsilon (optimality precision)" << std::endl
  			<< "  -d d, --discount-factor=d\t"
  			<< "discount factor (value function discount)" << std::endl
  			<< "  -p p, --probabilistic-threshold=p\t"
  			<< "probabilistic threshold for PSP* or TrFSP*" << std::endl
  			<< "  -N n, --number-of-particles=n\t"
  			<< "number of particles for PSP*" << std::endl
  			<< "  -T t, --trajectories-max-depth=t\t"
  			<< "Maximum depth of trajectories for PSP* or (L)RTDP ; lookahead for BFDT" << std::endl
  			<< "  -t t, --time-threshold=t\t"
  			<< "time threshold (timeout limit in milliseconds)" << std::endl
  			<< "  -n n, --trials-number=n\t"
  			<< "number of simulation trials" << std::endl
  			<< "  -m m, --trials-max-depth=m\t"
  			<< "maximum depth of simulation trials" << std::endl
  			<< "  -l l, --goals-limit=l\t"
  			<< "limit of goals number for subgoals generation algorithms" << std::endl
  			<< "  -q q, --deterministic-probability-weight=q\t"
  			<< "weight of the (log) probability fluent in the deterministic planner's goal formula (for reward-based MDPs)" << std::endl
            << "  -r r, --deterministic-reward-weight=r\t"
            << "weight of the reward fluent in the deterministic planner's goal formula (for reward-based MDPs)" << std::endl
            << "  -x x, --deterministic-planner-timeout=x\t"
            << "timeout (in seconds) of the deterministic planner's engine" << std::endl
            << "  -X X, --deterministic-planner-max-trials=X\t"
            << "maximum number of trials for the deterministic planner to find a solution (for reward-based MDPs)" << std::endl
            << "  -u u, --proportion-updated-states=u\t"
            << "proportion of updated states used by the MVPFPI and MVHOPI algorithms" << std::endl
            << "  -a a, --minimum-mean=a\t"
			<< "minimum mean allowed for mean-variance optimization"
			<< "  -b b, --maximum-variance=b\t"
			<< "maximum variance allowed for mean-variance optimization"
			<< "  -O --optimize-reward-determinization\t"
			<< "optimize the current policy constructed by reward-based robust determinization algorithms (grdrd or grdro)"
            << "  -v[n], --verbose[=n]\t"
            << "use verbosity level n;" << std::endl
            << "\t\t\t  n is a number from 0 (verbose mode off) and up;"
            << std::endl
            << "\t\t\t  default level is 1 if optional argument is left out"
            << std::endl
            << "  -W[n], --warnings[=n]\t"
            << "determines how warnings are treated;" << std::endl
            << "\t\t\t  0 suppresses warnings; 1 displays warnings;"
            << std::endl
            << "\t\t\t  2 treats warnings as errors" << std::endl
            << "  -h     --help\t\t"
            << "display this help and exit" << std::endl
            << "  file ...\t\t"
            << "files containing domain and problem descriptions;" << std::endl
            << "\t\t\t  if none, descriptions are read from standard input"
            << std::endl
            << std::endl
            << "Report bugs to <" PACKAGE_BUGREPORT ">." << std::endl;
}


/* Parses the given file, and returns true on success. */
static bool read_file(const char* name) {
  yyin = fopen(name, "r");
  if (yyin == 0) {
    std::cerr << "plannertest:" << name << ": " << strerror(errno)
              << std::endl;
    return false;
  } else {
    current_file = name;
    bool success = (yyparse() == 0);
    fclose(yyin);
    return success;
  }
}


static void run_algorithm(heuristic_t heuristic, algorithm_t algorithm, encoding_t encoding,
		determinization_t determinization, deterministic_planner_t deterministic_planner, test_planner_t test_type,
		const Problem& pb, double epsilon, double discount_factor,
		long time_threshold, unsigned int trials_number, unsigned int trials_max_depth,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth, unsigned int goals_limit,
		double deterministic_probability_weight, double deterministic_reward_weight,
        unsigned int deterministic_planner_timeout, unsigned int deterministic_planner_max_trials,
		double proportion_updated_states, double minimum_mean, double maximum_variance)
{
	TestPlanner* p = NULL;

	switch (test_type)
	{
	case TEST_PLANNER_COMPLETE :
		p = new CompleteTestPlanner(pb, algorithm, heuristic, encoding, determinization, deterministic_planner, epsilon, discount_factor,
			time_threshold, trials_number, trials_max_depth, probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
			deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
			proportion_updated_states, minimum_mean, maximum_variance, cgs_flag);
		break;

	case TEST_PLANNER_ANYTIME :
		p = new AnytimeTestPlanner(pb, algorithm, heuristic, encoding, determinization, deterministic_planner, epsilon, discount_factor,
			time_threshold, trials_number, trials_max_depth, probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
			deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
			proportion_updated_states, minimum_mean, maximum_variance, cgs_flag);
		break;
	}

	TestPlanner::SimulationInfo siminfo = p->launch();
	std::cerr << "      total solving time: " << (p->get_solving_time() / 1000.0) << " seconds" << std::endl;
	std::cerr << "      per-trial solving time: " << (siminfo.average_time_per_trial_ / 1000.0) << " seconds" << std::endl;
	std::cerr << "      value: " << siminfo.average_cumulated_value_ << std::endl;
	std::cerr << "      goal reached: " << siminfo.goal_reached_percentage_ << "%" << std::endl;
	std::cerr << "      turns: " << siminfo.average_turns_number_ << std::endl;
	std::cerr << "      reachable state space size: " << (p->get_algorithm().reachable_state_space_size()) << std::endl;
	std::cerr << "      (re)plannings: " << siminfo.nb_of_replannings_ << std::endl;
	std::cerr << "      average planning time: " << siminfo.average_planning_time_ << std::endl;
	std::cerr << "      time out: " << siminfo.time_out_percentage_ << "%" << std::endl;
	delete p;
}


static void run_dp_tests(heuristic_t heuristic, determinization_t determinization, deterministic_planner_t deterministic_planner,
		test_planner_t test_type, const Problem& pb, double epsilon, double discount_factor,
		long time_threshold, unsigned int trials_number, unsigned int trials_max_depth,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth, unsigned int goals_limit,
		double deterministic_probability_weight, double deterministic_reward_weight,
        unsigned int deterministic_planner_timeout, unsigned int deterministic_planner_max_trials,
		double proportion_updated_states, double minimum_mean, double maximum_variance, bool optimize_reward_determinization, bool heuristic_algorithms)
{
	if (!heuristic_algorithms)
	{
		if (grdpg_flag)
		{
			std::cerr << "   RDPG [GRAPH]" << std::endl;
			run_algorithm(heuristic, ALGORITHM_RDPG, ENCODING_GRAPH, determinization, deterministic_planner, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
					probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
					deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
					proportion_updated_states, minimum_mean, maximum_variance);
		}

		if (grdrg_flag)
		{
			std::cerr << "   RDRG [GRAPH]" << std::endl;
			run_algorithm(heuristic, ALGORITHM_RDRG, ENCODING_GRAPH, determinization, deterministic_planner, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
					probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
					deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
					proportion_updated_states, minimum_mean, maximum_variance);
		}

		if (grdbg_flag)
		{
			std::cerr << "   RDBG [GRAPH]" << std::endl;
			run_algorithm(heuristic, ALGORITHM_RDBG, ENCODING_GRAPH, determinization, deterministic_planner, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
					probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
					deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
					proportion_updated_states, minimum_mean, maximum_variance);
		}

		if (grdrd_flag && !optimize_reward_determinization)
		{
			std::cerr << "   RDRD [GRAPH]" << std::endl;
			run_algorithm(heuristic, ALGORITHM_RDRD, ENCODING_GRAPH, determinization, deterministic_planner, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
					probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
					deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
					proportion_updated_states, minimum_mean, maximum_variance);
		}

		if (grdrd_flag && optimize_reward_determinization)
		{
			std::cerr << "   OPTIMIZED-RDRD [GRAPH]" << std::endl;
			run_algorithm(heuristic, ALGORITHM_RDRD_STAR, ENCODING_GRAPH, determinization, deterministic_planner, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
					probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
					deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
					proportion_updated_states, minimum_mean, maximum_variance);
		}

		if (grdro_flag && !optimize_reward_determinization)
		{
			std::cerr << "   RDRO [GRAPH]" << std::endl;
			run_algorithm(heuristic, ALGORITHM_RDRO, ENCODING_GRAPH, determinization, deterministic_planner, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
					probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
					deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
					proportion_updated_states, minimum_mean, maximum_variance);
		}

		if (grdro_flag && optimize_reward_determinization)
		{
			std::cerr << "   OPTIMIZED-RDRO [GRAPH]" << std::endl;
			run_algorithm(heuristic, ALGORITHM_RDRO_STAR, ENCODING_GRAPH, determinization, deterministic_planner, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
					probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
					deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
					proportion_updated_states, minimum_mean, maximum_variance);
		}
	}

	if (heuristic_algorithms && god_flag)
	{
		std::cerr << "   OD [GRAPH]" << std::endl;
		run_algorithm(heuristic, ALGORITHM_OD, ENCODING_GRAPH, determinization, deterministic_planner, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}
}


static void run_deterministic_planner_tests(heuristic_t heuristic, determinization_t determinization, test_planner_t test_type,
		const Problem& pb, double epsilon, double discount_factor,
		long time_threshold, unsigned int trials_number, unsigned int trials_max_depth,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth, unsigned int goals_limit,
		double deterministic_probability_weight, double deterministic_reward_weight,
        unsigned int deterministic_planner_timeout, unsigned int deterministic_planner_max_trials,
		double proportion_updated_states, double minimum_mean, double maximum_variance, bool optimized_reward_determinization, bool heuristic_algorithms)
{
	if (dpff_flag)
	{
		std::cerr << "****** DETERMINISTIC PLANNER: FF *******" << std::endl;
		run_dp_tests(heuristic, determinization, DETERMINISTIC_PLANNER_FF, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization, heuristic_algorithms);
	}

	if (dpmff_flag)
	{
		std::cerr << "****** DETERMINISTIC PLANNER: METRIC-FF *******" << std::endl;
		run_dp_tests(heuristic, determinization, DETERMINISTIC_PLANNER_MFF, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization, heuristic_algorithms);
	}
}


static void run_determinization_algorithm_tests(heuristic_t heuristic, test_planner_t test_type, const Problem& pb, double epsilon, double discount_factor,
		long time_threshold, unsigned int trials_number, unsigned int trials_max_depth,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth, unsigned int goals_limit,
		double deterministic_probability_weight, double deterministic_reward_weight,
        unsigned int deterministic_planner_timeout, unsigned int deterministic_planner_max_trials,
		double proportion_updated_states, double minimum_mean, double maximum_variance, bool optimized_reward_determinization, bool heuristic_algorithms)
{
	if (dmpo_flag && !grdrd_flag && !grdro_flag)
	{
		std::cerr << "-DETERMINIZATION: MOST PROBABLE OUTCOME [GOAL]-" << std::endl;
		run_deterministic_planner_tests(heuristic, DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization, heuristic_algorithms);
	}

	if (dmpo_flag && (grdrd_flag || grdro_flag))
	{
		std::cerr << "-DETERMINIZATION: MOST PROBABLE OUTCOME [REWARD]-" << std::endl;
		run_deterministic_planner_tests(heuristic, DETERMINIZATION_MOST_PROBABLE_OUTCOME_REWARD, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization, heuristic_algorithms);
	}

	if (dao_flag && !grdrd_flag && !grdro_flag)
	{
		std::cerr << "-DETERMINIZATION: ALL OUTCOMES [GOAL]-" << std::endl;
		run_deterministic_planner_tests(heuristic, DETERMINIZATION_ALL_OUTCOMES_GOAL, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization, heuristic_algorithms);
	}

	if (dao_flag && (grdrd_flag || grdro_flag))
	{
		std::cerr << "-DETERMINIZATION: ALL OUTCOMES [REWARD]-" << std::endl;
		run_deterministic_planner_tests(heuristic, DETERMINIZATION_ALL_OUTCOMES_REWARD, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization, heuristic_algorithms);
	}
}


static void run_heuristics_deterministic_planner_tests(heuristic_t heuristic, algorithm_t algorithm, encoding_t encoding,
		determinization_t determinization, test_planner_t test_type, const Problem& pb, double epsilon, double discount_factor,
		long time_threshold, unsigned int trials_number, unsigned int trials_max_depth,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth, unsigned int goals_limit,
		double deterministic_probability_weight, double deterministic_reward_weight,
        unsigned int deterministic_planner_timeout, unsigned int deterministic_planner_max_trials,
		double proportion_updated_states, double minimum_mean, double maximum_variance)
{
	if (dpff_flag)
	{
		std::cerr << "****** DETERMINISTIC PLANNER: FF *******" << std::endl;
		run_algorithm(heuristic, algorithm, encoding, determinization, DETERMINISTIC_PLANNER_FF, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (dpmff_flag)
	{
		std::cerr << "****** DETERMINISTIC PLANNER: METRIC-FF *******" << std::endl;
		run_algorithm(heuristic, algorithm, encoding, determinization, DETERMINISTIC_PLANNER_MFF, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}
}


static void run_determinization_heuristics_tests(heuristic_t heuristic, algorithm_t algorithm, encoding_t encoding,
		test_planner_t test_type, const Problem& pb, double epsilon, double discount_factor,
		long time_threshold, unsigned int trials_number, unsigned int trials_max_depth,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth, unsigned int goals_limit,
		double deterministic_probability_weight, double deterministic_reward_weight,
        unsigned int deterministic_planner_timeout, unsigned int deterministic_planner_max_trials,
		double proportion_updated_states, double minimum_mean, double maximum_variance)
{
	if (hdet_flag && dmpo_flag && !grdrd_flag && !grdro_flag)
	{
		std::cerr << "-DETERMINIZATION: MOST PROBABLE OUTCOME [GOAL]-" << std::endl;
		run_heuristics_deterministic_planner_tests(heuristic, algorithm, encoding, DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL,
				test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (hdet_flag && dmpo_flag && (grdrd_flag || grdro_flag))
	{
		std::cerr << "-DETERMINIZATION: MOST PROBABLE OUTCOME [REWARD]-" << std::endl;
		run_heuristics_deterministic_planner_tests(heuristic, algorithm, encoding, DETERMINIZATION_MOST_PROBABLE_OUTCOME_REWARD,
				test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (hdet_flag && dao_flag && !grdrd_flag && !grdro_flag)
	{
		std::cerr << "-DETERMINIZATION: ALL OUTCOMES [GOAL]-" << std::endl;
		run_heuristics_deterministic_planner_tests(heuristic, algorithm, encoding, DETERMINIZATION_ALL_OUTCOMES_GOAL,
				test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (hdet_flag && dao_flag && (grdrd_flag || grdro_flag))
	{
		std::cerr << "-DETERMINIZATION: ALL OUTCOMES [REWARD]-" << std::endl;
		run_heuristics_deterministic_planner_tests(heuristic, algorithm, encoding, DETERMINIZATION_ALL_OUTCOMES_REWARD,
				test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (hzero_flag || hpstep_flag || hnstep_flag || hrstep_flag || hrrewards_flag || hrbounds_flag || hebounds_flag || hmax_flag || hadd_flag)
	{
		run_algorithm(heuristic, algorithm, encoding, DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL, DETERMINISTIC_PLANNER_FF,
				test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}
}


static void run_heuristics_tests(heuristic_t heuristic, test_planner_t test_type, const Problem& pb, double epsilon, double discount_factor,
		long time_threshold, unsigned int trials_number, unsigned int trials_max_depth,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth, unsigned int goals_limit,
		double deterministic_probability_weight, double deterministic_reward_weight,
        unsigned int deterministic_planner_timeout, unsigned int deterministic_planner_max_trials,
		double proportion_updated_states, double minimum_mean, double maximum_variance, bool optimized_reward_determinization)
{
	if (gfsp_flag)
	{
		std::cerr << "   FSP [GRAPH]" << std::endl;
		run_determinization_heuristics_tests(heuristic, ALGORITHM_FSP, ENCODING_GRAPH, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (sfsp_flag)
	{
		std::cerr << "   FSP [SYMBOLIC]" << std::endl;
		run_determinization_heuristics_tests(heuristic, ALGORITHM_FSP, ENCODING_SYMBOLIC, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (gfsps_flag)
	{
		std::cerr << "   FSP* [GRAPH]" << std::endl;
		run_determinization_heuristics_tests(heuristic, ALGORITHM_FSP_STAR, ENCODING_GRAPH, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (sfsps_flag)
	{
		std::cerr << "   FSP* [SYMBOLIC]" << std::endl;
		run_determinization_heuristics_tests(heuristic, ALGORITHM_FSP_STAR, ENCODING_SYMBOLIC, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (gtrfsps_flag)
	{
		std::cerr << "   TrFSP* [GRAPH]" << std::endl;
		run_determinization_heuristics_tests(heuristic, ALGORITHM_TRFSP_STAR, ENCODING_GRAPH, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (gbfdt_flag)
	{
		std::cerr << "   BFDT [GRAPH]" << std::endl;
		run_determinization_heuristics_tests(heuristic, ALGORITHM_BFDT, ENCODING_GRAPH, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (sbfdt_flag)
	{
		std::cerr << "   BFDT [SYMBOLIC]" << std::endl;
		run_determinization_heuristics_tests(heuristic, ALGORITHM_BFDT, ENCODING_SYMBOLIC, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (gpsps_flag)
	{
		std::cerr << "   PSP* [GRAPH]" << std::endl;
		run_determinization_heuristics_tests(heuristic, ALGORITHM_PSP_STAR, ENCODING_GRAPH, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (gfdp_flag)
	{
		std::cerr << "   FDP [GRAPH]" << std::endl;
		run_determinization_heuristics_tests(heuristic, ALGORITHM_FDP, ENCODING_GRAPH, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (glao_flag)
	{
		std::cerr << "   LAO* [GRAPH]" << std::endl;
		run_determinization_heuristics_tests(heuristic, ALGORITHM_LAO, ENCODING_GRAPH, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (slao_flag)
	{
		std::cerr << "   LAO* [SYMBOLIC]" << std::endl;
		run_determinization_heuristics_tests(heuristic, ALGORITHM_LAO, ENCODING_SYMBOLIC, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (gilao_flag)
	{
		std::cerr << "   Improved LAO* [GRAPH]" << std::endl;
		run_determinization_heuristics_tests(heuristic, ALGORITHM_IMPROVED_LAO, ENCODING_GRAPH, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (grtdp_flag)
	{
		std::cerr << "   RTDP [GRAPH]" << std::endl;
		run_determinization_heuristics_tests(heuristic, ALGORITHM_RTDP, ENCODING_GRAPH, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (srtdp_flag)
	{
		std::cerr << "   RTDP [SYMBOLIC]" << std::endl;
		run_determinization_heuristics_tests(heuristic, ALGORITHM_RTDP, ENCODING_SYMBOLIC, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (glrtdp_flag)
	{
		std::cerr << "   LRTDP [GRAPH]" << std::endl;
		run_determinization_heuristics_tests(heuristic, ALGORITHM_LRTDP, ENCODING_GRAPH, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (slrtdp_flag)
	{
		std::cerr << "   LRTDP [SYMBOLIC]" << std::endl;
		run_determinization_heuristics_tests(heuristic, ALGORITHM_LRTDP, ENCODING_SYMBOLIC, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (gldfs_flag)
	{
		std::cerr << "   LDFS [GRAPH]" << std::endl;
		run_determinization_heuristics_tests(heuristic, ALGORITHM_LDFS, ENCODING_GRAPH, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (sldfs_flag)
	{
		std::cerr << "   LDFS [SYMBOLIC]" << std::endl;
		run_determinization_heuristics_tests(heuristic, ALGORITHM_LDFS, ENCODING_SYMBOLIC, test_type,
				pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance);
	}

	if (god_flag)
	{
		run_determinization_algorithm_tests(heuristic, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
					probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
					deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
					proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization, true);
	}
}


static void run_tests(test_planner_t test_type, const Problem& pb, double epsilon, double discount_factor,
		long time_threshold, unsigned int trials_number, unsigned int trials_max_depth,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth, unsigned int goals_limit,
		double deterministic_probability_weight, double deterministic_reward_weight,
        unsigned int deterministic_planner_timeout, unsigned int deterministic_planner_max_trials,
		double proportion_updated_states, double minimum_mean, double maximum_variance, bool optimized_reward_determinization)
{
	if (gvi_flag || svi_flag || grdpg_flag || grdrg_flag || grdbg_flag || grdrd_flag || grdro_flag || ggco_flag || sgco_flag || gmvpfpi_flag || gmvhopi_flag || gmvpfas_flag || gmvhoas_flag || gmvpfssp_flag)
	{
		std::cerr << "----------------------------------------" << std::endl;
		std::cerr << "--- NO HEURISTIC -----------------------" << std::endl;
		std::cerr << "----------------------------------------" << std::endl;

		if (gvi_flag)
		{
			std::cerr << "   VI [GRAPH]" << std::endl;
			run_algorithm(HEURISTIC_ZERO, ALGORITHM_VI, ENCODING_GRAPH, DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL, DETERMINISTIC_PLANNER_FF,
					test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
					probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
					deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
					proportion_updated_states, minimum_mean, maximum_variance);
		}

		if (svi_flag)
		{
			std::cerr << "   VI [SYMBOLIC]" << std::endl;
			run_algorithm(HEURISTIC_ZERO, ALGORITHM_VI, ENCODING_SYMBOLIC, DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL, DETERMINISTIC_PLANNER_FF,
					test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
					probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
					deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
					proportion_updated_states, minimum_mean, maximum_variance);
		}

		if (grdpg_flag || grdrg_flag || grdbg_flag || grdrd_flag || grdro_flag)
		{
			run_determinization_algorithm_tests(HEURISTIC_ZERO, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
					probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
					deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
					proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization, false);
		}

        if (ggco_flag)
        {
            std::cerr << "   GCO [GRAPH]" << std::endl;
			run_algorithm(HEURISTIC_ZERO, ALGORITHM_GCO, ENCODING_GRAPH, DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL, DETERMINISTIC_PLANNER_FF,
					test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
					probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
					deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
					proportion_updated_states, minimum_mean, maximum_variance);
        }

		if (sgco_flag)
        {
            std::cerr << "   GCO [SYMBOLIC]" << std::endl;
			run_algorithm(HEURISTIC_ZERO, ALGORITHM_GCO, ENCODING_SYMBOLIC, DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL, DETERMINISTIC_PLANNER_FF,
					test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
					probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
					deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
					proportion_updated_states, minimum_mean, maximum_variance);
        }

        if (gmvpfpi_flag)
        {
			std::cerr << "   MVPFPI [GRAPH]" << std::endl;
			run_algorithm(HEURISTIC_ZERO, ALGORITHM_MVPFPI, ENCODING_GRAPH, DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL, DETERMINISTIC_PLANNER_FF,
					test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
					probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
					deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
					proportion_updated_states, minimum_mean, maximum_variance);
		}

		if (gmvhopi_flag)
        {
			std::cerr << "   MVHOPI [GRAPH]" << std::endl;
			run_algorithm(HEURISTIC_ZERO, ALGORITHM_MVHOPI, ENCODING_GRAPH, DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL, DETERMINISTIC_PLANNER_FF,
					test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
					probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
					deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
					proportion_updated_states, minimum_mean, maximum_variance);
		}

		if (gmvpfas_flag)
        {
			std::cerr << "   MVPFAS [GRAPH]" << std::endl;
			run_algorithm(HEURISTIC_ZERO, ALGORITHM_MVPFAS, ENCODING_GRAPH, DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL, DETERMINISTIC_PLANNER_FF,
					test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
					probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
					deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
					proportion_updated_states, minimum_mean, maximum_variance);
		}

		if (gmvhoas_flag)
        {
			std::cerr << "   MVHOAS [GRAPH]" << std::endl;
			run_algorithm(HEURISTIC_ZERO, ALGORITHM_MVHOAS, ENCODING_GRAPH, DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL, DETERMINISTIC_PLANNER_FF,
					test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
					probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
					deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
					proportion_updated_states, minimum_mean, maximum_variance);
		}

		if (gmvpfssp_flag)
        {
			std::cerr << "   MVPFSSP [GRAPH]" << std::endl;
			run_algorithm(HEURISTIC_ZERO, ALGORITHM_MVPFSSP, ENCODING_GRAPH, DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL, DETERMINISTIC_PLANNER_FF,
					test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
					probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
					deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
					proportion_updated_states, minimum_mean, maximum_variance);
		}
	}

	if (hzero_flag)
	{
		std::cerr << "----------------------------------------" << std::endl;
		std::cerr << "--- HEURISTIC: ZERO --------------------" << std::endl;
		std::cerr << "----------------------------------------" << std::endl;
		run_heuristics_tests(HEURISTIC_ZERO, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization);
	}

	if (hpstep_flag)
	{
		std::cerr << "----------------------------------------" << std::endl;
		std::cerr << "--- HEURISTIC: PDDL STEP DISTANCE ------" << std::endl;
		std::cerr << "----------------------------------------" << std::endl;
		run_heuristics_tests(HEURISTIC_PDDL_STEP_DISTANCE, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization);
	}

	if (hnstep_flag)
	{
		std::cerr << "----------------------------------------" << std::endl;
		std::cerr << "--- HEURISTIC: NATIVE STEP DISTANCE ----" << std::endl;
		std::cerr << "----------------------------------------" << std::endl;
		run_heuristics_tests(HEURISTIC_NATIVE_STEP_DISTANCE, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization);
	}

	if (hrstep_flag)
	{
		std::cerr << "----------------------------------------" << std::endl;
		std::cerr << "--- HEURISTIC: RELAXED STEP DISTANCE ---" << std::endl;
		std::cerr << "----------------------------------------" << std::endl;
		run_heuristics_tests(HEURISTIC_RELAXED_STEP_DISTANCE, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization);
	}

	if (hrrewards_flag)
	{
		std::cerr << "----------------------------------------" << std::endl;
		std::cerr << "------ HEURISTIC: RELAXED REWARDS ------" << std::endl;
		std::cerr << "----------------------------------------" << std::endl;
		run_heuristics_tests(HEURISTIC_RELAXED_REWARDS, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization);
	}

	if (hrbounds_flag)
	{
		std::cerr << "----------------------------------------" << std::endl;
		std::cerr << "------ HEURISTIC: RELAXED BOUNDS -------" << std::endl;
		std::cerr << "----------------------------------------" << std::endl;
		run_heuristics_tests(HEURISTIC_RELAXED_PLANNING_GRAPH, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization);
	}

	if (hebounds_flag)
	{
		std::cerr << "----------------------------------------" << std::endl;
		std::cerr << "------- HEURISTIC: EXACT BOUNDS --------" << std::endl;
		std::cerr << "----------------------------------------" << std::endl;
		run_heuristics_tests(HEURISTIC_BOUNDS, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization);
	}

	if (hmax_flag)
	{
		std::cerr << "----------------------------------------" << std::endl;
		std::cerr << "------------ HEURISTIC: HMAX -----------" << std::endl;
		std::cerr << "----------------------------------------" << std::endl;
		run_heuristics_tests(HEURISTIC_HMAX, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization);
	}

	if (hadd_flag)
	{
		std::cerr << "----------------------------------------" << std::endl;
		std::cerr << "------------ HEURISTIC: HADD -----------" << std::endl;
		std::cerr << "----------------------------------------" << std::endl;
		run_heuristics_tests(HEURISTIC_HADD, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization);
	}

	if (hdet_flag)
	{
		std::cerr << "----------------------------------------" << std::endl;
		std::cerr << "------------ HEURISTIC: HDET -----------" << std::endl;
		std::cerr << "----------------------------------------" << std::endl;
		run_heuristics_tests(HEURISTIC_HDET, test_type, pb, epsilon, discount_factor, time_threshold, trials_number, trials_max_depth,
				probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
				deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization);
	}
}


int main(int argc, char **argv)
{
  /* no default problem */
  current_problem_hack_ = NULL;
  /* Set default verbosity. */
  verbosity = 0;
  /* Set default warning level. */
  warning_level = 1;
  /* Test mode */
  test_planner_t test_type = TEST_PLANNER_COMPLETE;
  /* Epsilon */
  double eps = 0.000001;
  /* Discount factor */
  double df = 0.9;
  /* Probabilistic threshold */
  double probabilistic_threshold = 0.1;
  /* Number of particles */
  unsigned int nb_of_particles = 10;
  /* Maximum depth of trajectories */
  unsigned int trajectories_max_depth = 100;
  /* Goals limit */
  unsigned int goals_limit = 100;
  /* Weight of the (log) probability fluent in the deterministic planner's goal formula (for reward-based MDPs) */
  double deterministic_probability_weight = 1.0;
  /* Weight of the reward fluent in the deterministic planner's goal formula (for reward-based MDPs) */
  double deterministic_reward_weight = 10.0;
  /* Timeout (in seconds) of the deterministic planner's engine */
  unsigned int deterministic_planner_timeout = 5;
  /* Maximum number of trials for the deterministic planner to find a solution (for reward-based MDPs) */
  unsigned int deterministic_planner_max_trials = 10;
  /* Proportion of updated states */
  double proportion_updated_states = 0.01;
  /* Minimum minimum allowed for mean-variance optimization */
  double minimum_mean = 0.0;
  /* Maximum variance allowed for mean-variance optimization */
  double maximum_variance = 0.0;
  /* Time threshold (in milliseconds) */
  long tt = 900000;
  /* Trials number */
  unsigned int tn = 100;
  /* Trials max depth */
  unsigned int tmd = 1000;
  /* Optional optimization of current policy in reward-based determinization algorithms */
  bool optimized_reward_determinization = false;

  try {
    /*
     * Get command line options.
     */
    while (1) {
      int option_index = 0;
      int c = getopt_long(argc, argv, OPTION_STRING,
                          long_options, &option_index);
      if (c == -1) {
        break;
      }
      switch (c) {
      case 0:
    	break;
      case 'M':
        if (strcmp(optarg, "complete") == 0)
          test_type = TEST_PLANNER_COMPLETE;
        else if (strcmp(optarg, "anytime") == 0)
          test_type = TEST_PLANNER_ANYTIME;
        else {
          std::cerr << "Unknown test mode. Try `plannertest --help' for more information."
    		   << std::endl;
    	  return -1;
        }
        break;
      case 'e':
    	eps = (optarg != 0) ? atof(optarg) : 0.000001;
    	break;
      case 'd':
    	df = (optarg != 0) ? atof(optarg) : 0.9;
    	break;
      case 'p':
    	probabilistic_threshold = (optarg != 0) ? atof(optarg) : 0.1;
    	break;
      case 'N':
    	nb_of_particles = (optarg != 0) ? atol(optarg) : 10;
    	break;
      case 'T':
    	trajectories_max_depth = (optarg != 0) ? atol(optarg) : 100;
    	break;
      case 't':
	tt = (optarg != 0) ? atol(optarg) : 900000;
	break;
      case 'n':
	tn = (optarg != 0) ? atol(optarg) : 100;
	break;
      case 'm':
	tmd = (optarg != 0) ? atol(optarg) : 1000;
	break;
      case 'l':
	goals_limit = (optarg != 0) ? atol(optarg) : 100;
	break;
	  case 'q':
    deterministic_probability_weight = (optarg != 0) ? atof(optarg) : 1.0;
    break;
      case 'r':
    deterministic_reward_weight = (optarg != 0) ? atof(optarg) : 10.0;
    break;
      case 'x':
    deterministic_planner_timeout = (optarg != 0) ? atol(optarg) : 5;
    break;
      case 'X':
    deterministic_planner_max_trials = (optarg != 0) ? atol(optarg) : 10;
    break;
	  case 'u':
	proportion_updated_states = (optarg != 0) ? atof(optarg) : 0.01;
	break;
	  case 'a':
	minimum_mean = (optarg != 0) ? atof(optarg) : 0.0;
	break;
	  case 'b':
	maximum_variance = (optarg != 0) ? atof(optarg) : 0.0;
	break;
	  case 'O':
    optimized_reward_determinization = true;
    break;
      case 'v':
        verbosity = (optarg != 0) ? atoi(optarg) : 1;
        break;
      case 'W':
        warning_level = (optarg != 0) ? atoi(optarg) : 1;
        break;
      case 'h':
        display_help();
        return 0;
      case ':':
      default:
        std::cerr << "Try `plannertest --help' for more information."
                  << std::endl;
        return -1;
      }
    }

    /*
     * Read pddl files.
     */
    if (optind < argc) {
      /*
       * Use remaining command line arguments as file names.
       */
      while (optind < argc) {
        if (!read_file(argv[optind++])) {
          return -1;
        }
      }
    } else {
      /*
       * No remaining command line argument, so read from standard input.
       */
      yyin = stdin;
      if (yyparse() != 0) {
        return -1;
      }
    }

    if (verbosity > 1) {
      /*
       * Display domains and problems.
       */
      std::cerr << "----------------------------------------"<< std::endl
                << "domains:" << std::endl;
      for (Domain::DomainMap::const_iterator di = Domain::begin();
           di != Domain::end(); di++) {
        std::cerr << std::endl << *(*di).second << std::endl;
      }
      std::cerr << "----------------------------------------"<< std::endl
                << "problems:" << std::endl;
      for (Problem::ProblemMap::const_iterator pi = Problem::begin();
           pi != Problem::end(); pi++) {
        std::cerr << std::endl << *(*pi).second << std::endl;
      }
      std::cerr << "----------------------------------------"<< std::endl;
    }

    for (Problem::ProblemMap::const_iterator pi = Problem::begin();
         pi != Problem::end(); pi++) {
      current_problem_hack_ = pi->second;
      run_tests(test_type, *(pi->second), eps, df, tt, tn, tmd, probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
                deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
				proportion_updated_states, minimum_mean, maximum_variance, optimized_reward_determinization);
    }
  } catch (const BaseException& e) {
    std::cerr << std::endl << "plannertest: " << e.what() << std::endl;
    return 1;
  } catch (const std::exception& e) {
    std::cerr << std::endl << "plannertest: " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "plannertest: fatal error" << std::endl;
    return -1;
  }

  Problem::clear();
  Domain::clear();

  return 0;
}
