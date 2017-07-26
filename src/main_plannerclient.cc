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
#include "client/ippc_client.h"
#include "states.h"
#include "problems.h"
#include "domains.h"
#include "actions.h"
#include <iostream>
#include <cerrno>
#include <cstdio>
#if HAVE_GETOPT_LONG
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <getopt.h>
#else
#include "port/getopt.h"
#endif
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>

#include "planners/rddl_planner.h"
#include "planners/complete_planner.h"
#include "planners/anytime_planner.h"
#include "algorithms/heuristic_algorithm.h"
#include "commons/base_exceptions.h"

typedef enum
{
	PLANNER_COMPLETE,
	PLANNER_ANYTIME
}
planner_t;


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
/* client name */
static std::string client_name;
/* Current problem hack */
const Problem* current_problem_hack_;

/* Program options. */
static struct option long_options[] = {
  { "host", required_argument, 0, 'H' },
  { "port", required_argument, 0, 'P' },
  { "client-name", optional_argument, 0, 'C'},
  { "planner", required_argument, 0, 'R' },
  { "algorithm", required_argument, 0, 'A' },
  { "heuristic", optional_argument, 0, 'a' },
  { "encoding", required_argument, 0, 'E' },
  { "determinization", optional_argument, 0, 'D' },
  { "deterministic-planner", optional_argument, 0, 'F' },
  { "planning-time", optional_argument, 0, 't' },
  { "ippc", no_argument, 0, 'I' },
  { "epsilon", optional_argument, 0, 'e' },
  { "discount-factor", optional_argument, 0, 'd' },
  { "probabilistic-threshold", optional_argument, 0, 'p' },
  { "number-of-particles", optional_argument, 0, 'N' },
  { "trajectories_max_depth", optional_argument, 0, 'T' },
  { "goals-limit", optional_argument, 0, 'l' },
  { "deterministic-probability-weight", optional_argument, 0, 'q' },
  { "deterministic-reward-weight", optional_argument, 0, 'r' },
  { "deterministic-planner-timeout", optional_argument, 0, 'x' },
  { "deterministic-planner-max-trials", optional_argument, 0, 'X' },
  { "optimize-reward-determinization", no_argument, 0, 'O' },
  { "verbose", optional_argument, 0, 'v' },
  { "warnings", optional_argument, 0, 'W' },
  { "help", no_argument, 0, 'h' },
  { 0, 0, 0, 0 }
};
static const char OPTION_STRING[] = "OH:P:C:R:A:a:E:D:F:t:e:d:p:N:IT:l:q:r:x:X:v::W::h";


/* Displays help. */
static void display_help() {
  std::cout << "usage: plannerclient [options] [file ...]" << std::endl
            << "options:" << std::endl
            << "  -H h,  --host=h\t"
            << "connect to host h" << std::endl
            << "  -P p,  --port=p\t"
            << "connect to port p" << std::endl
            << "  -C c,  --client-name=c\t"
            << "client name (default: plannerclient)" << std::endl
            << "  -R r,  --planner=r\t"
            << "run planner r (complete or anytime)" << std::endl
            << "  -A a,  --algorithm=a\t"
            << "run algorithm a (fsp or fsp-star or trfsp-star or bfdt or psp-star or fdp or vi or lao or ilao or rtdp or lrtdp or ldfs or od or rdpg or rdrg or rdbg or rdrd or rdro)" << std::endl
            << "  -a a,  --heuristic=a\t"
            << "use heuristic a (zero or pddl-step-distance or native-step-distance or relaxed-step-distance or relaxed-rewards or relaxed-bounds or exact-bounds or hmax or hadd or hdet)" << std::endl
  			<< "  -E e,  --encoding=e\t"
  			<< "encoding (graph or symbolic)" << std::endl
  			<< "  -D d,  --determinization=d\t"
  			<< "determinization (most-probable-outcome or all-outcomes)" << std::endl
  			<< "  -F f,   --deterministic-planner=f\t"
  			<< "deterministic planner (ff or mff)" << std::endl
  			<< "  -t t,  --planning-time=t"
  			<< "planning time for an anytime planner when no policy is available in the current state (in milliseconds)" << std::endl
  			<< "  -e e,  --epsilon=e\t"
  			<< "epsilon (optimality precision)" << std::endl
  			<< "  -d d,  --discount-factor=d\t"
  			<< "discount factor (value function discount)" << std::endl
  			<< "  -p p,  --probabilistic-threshold=p\t"
  			<< "probabilistic threshold for PSP* or TrFSP* or od or rdpg or rdrg or rdbg" << std::endl
  			<< "  -N n,  --number-of-particles=n\t"
  			<< "number of particles for PSP*" << std::endl
  			<< "  -T t,  --trajectories-max-depth=t\t"
  			<< "Maximum depth of trajectories for PSP* or (L)RTDP ; or lookahead for BFDT" << std::endl
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
            << "  -O --optimize-reward-determinization\t"
			<< "optimize the current policy constructed by reward-based robust determinization algorithms (rdrd or rdro)"
            << "  -v[n],  --verbose[=n]\t"
            << "use verbosity level n;" << std::endl
            << "\t\t\t  n is a number from 0 (verbose mode off) and up;"
            << std::endl
            << "\t\t\t  default level is 1 if optional argument is left out"
            << std::endl
            << "  -W[n], --warnings[=n]\t"
            << "determines how warnings are treated;" << std::endl
            << "\t\t\t  0 supresses warnings; 1 displays warnings;"
            << std::endl
            << "\t\t\t  2 treats warnings as errors" << std::endl
            << "  -I --ippc\t\t"
            << "uses the mdpsim IPPC client" << std::endl
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
    std::cerr << client_name << ':' << name << ": " << strerror(errno)
              << std::endl;
    return false;
  } else {
    current_file = name;
    bool success = (yyparse() == 0);
    fclose(yyin);
    return success;
  }
}

int connect(const char *hostname, int port)
{
  struct hostent *host = ::gethostbyname(hostname);
  if (!host) {
    perror("gethostbyname");
    return -1;
  }

  int sock = ::socket(PF_INET, SOCK_STREAM, 0);
  if (sock == -1) {
    perror("socket");
    return -1;
  }

  struct sockaddr_in addr;
  addr.sin_family=AF_INET;
  addr.sin_port=htons(port);
  addr.sin_addr = *((struct in_addr *)host->h_addr);
  memset(&(addr.sin_zero), '\0', 8);

//  if (::connect(sock, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
//    perror("connect");
//    return -1;
//  }
  while (::connect(sock, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
    usleep(500000);
  }
  return sock;
  //remember to call close(sock) when you're done
}


int main(int argc, char **argv)
{
  /* no default problem */
  current_problem_hack_ = NULL;
  /* Set default verbosity. */
  verbosity = 0;
  /* Set default warning level. */
  warning_level = 1;
  /* Host. */
  std::string host;
  /* Port. */
  int port = 0;
  /* Set default client name */
  client_name = "plannerclient";
  /* Use IPPC client from PROST */
  bool ippc = false;
  /* Planner */
  planner_t pln = PLANNER_COMPLETE;
  /* Algorithm */
  algorithm_t alg = ALGORITHM_FSP;
  /* Heuristic */
  heuristic_t heur = HEURISTIC_PDDL_STEP_DISTANCE;
  /* Encoding */
  encoding_t enc = ENCODING_GRAPH;
  /* Determinization */
  determinization_t determinization = DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL;
  /* Deterministic planner */
  deterministic_planner_t  deterministic_planner = DETERMINISTIC_PLANNER_FF;
  /* Planning time for an anytime planner */
  long planning_time = 60000;
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
      case 'H':
        host = optarg;
        break;
      case 'I':
        ippc = true;
        break;
      case 'P':
        port = atoi(optarg);
        break;
      case 'C':
    	client_name = optarg;
    	break;
      case 'R':
    	if (strcmp(optarg, "complete") == 0)
    		pln = PLANNER_COMPLETE;
    	else if (strcmp(optarg, "anytime") == 0)
    		pln = PLANNER_ANYTIME;
    	else {
    	  std::cerr << "Unknown planner. Try `plannerclient --help' for more information."
    		   << std::endl;
    	  return -1;
    	}
    	break;
      case 'A':
    	if (strcmp(optarg, "vi") == 0)
    	  alg = ALGORITHM_VI;
    	else if (strcmp(optarg, "fsp") == 0)
    		alg = ALGORITHM_FSP;
    	else if (strcmp(optarg, "fsp-star") == 0)
    		alg = ALGORITHM_FSP_STAR;
    	else if (strcmp(optarg, "trfsp-star") == 0)
    		alg = ALGORITHM_TRFSP_STAR;
		else if (strcmp(optarg, "bfdt") == 0)
			alg = ALGORITHM_BFDT;
    	else if (strcmp(optarg, "psp-star") == 0)
    		alg = ALGORITHM_PSP_STAR;
    	else if (strcmp(optarg, "fdp") == 0)
    		alg = ALGORITHM_FDP;
    	else if (strcmp(optarg, "lao") == 0)
    		alg = ALGORITHM_LAO;
    	else if (strcmp(optarg, "ilao") == 0)
    		alg = ALGORITHM_IMPROVED_LAO;
    	else if (strcmp(optarg, "rtdp") == 0)
    		alg = ALGORITHM_RTDP;
		else if (strcmp(optarg, "lrtdp") == 0)
    		alg = ALGORITHM_LRTDP;
		else if (strcmp(optarg, "ldfs") == 0)
    		alg = ALGORITHM_LDFS;
    	else if (strcmp(optarg, "od") == 0)
    		alg = ALGORITHM_OD;
    	else if (strcmp(optarg, "rdpg") == 0)
    		alg = ALGORITHM_RDPG;
    	else if (strcmp(optarg, "rdrg") == 0)
    		alg = ALGORITHM_RDRG;
    	else if (strcmp(optarg, "rdbg") == 0)
    		alg = ALGORITHM_RDBG;
        else if (strcmp(optarg, "rdrd") == 0)
    		alg = ALGORITHM_RDRD;
    	else if (strcmp(optarg, "rdro") == 0)
    		alg = ALGORITHM_RDRO;
    	else {
    	  std::cerr << "Unknown algorithm. Try `plannerclient --help' for more information."
    		   << std::endl;
    	  return -1;
    	}
    	break;
      case 'a':
    	if (strcmp(optarg, "zero") == 0)
    		heur = HEURISTIC_ZERO;
    	else if (strcmp(optarg, "pddl-step-distance") == 0)
    	  heur = HEURISTIC_PDDL_STEP_DISTANCE;
    	else if (strcmp(optarg, "native-step-distance") == 0)
    	  heur = HEURISTIC_NATIVE_STEP_DISTANCE;
    	else if (strcmp(optarg, "relaxed-step-distance") == 0)
    	  heur = HEURISTIC_RELAXED_STEP_DISTANCE;
    	else if (strcmp(optarg, "relaxed-rewards") == 0)
    	  heur = HEURISTIC_RELAXED_REWARDS;
    	else if (strcmp(optarg, "relaxed-bounds") == 0)
    	  heur = HEURISTIC_RELAXED_PLANNING_GRAPH;
    	else if (strcmp(optarg, "exact-bounds") == 0)
    		heur = HEURISTIC_BOUNDS;
		else if (strcmp(optarg, "hmax") == 0)
    	  heur = HEURISTIC_HMAX;
		else if (strcmp(optarg, "hadd") == 0)
    	  heur = HEURISTIC_HADD;
		else if (strcmp(optarg, "hdet") == 0)
    	  heur = HEURISTIC_HDET;
    	else {
    	  std::cerr << "Unknown heuristic. Try `plannerclient --help' for more information."
    		   << std::endl;
    	  return -1;
    	}
    	break;
      case 'E':
    	if (strcmp(optarg, "graph") == 0)
    	  enc = ENCODING_GRAPH;
    	else if (strcmp(optarg, "symbolic") == 0)
    	  enc = ENCODING_SYMBOLIC;
    	else {
    	  std::cerr << "Unknown encoding. Try `plannerclient --help' for more information."
    	    	<< std::endl;
    	  return -1;
    	}
    	break;
      case 'D':
    	if (strcmp(optarg, "most-probable-outcome") == 0)
    	  determinization = DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL; // to be changed after the 'while' option loop if reward-based MDP
    	else if (strcmp(optarg, "all-outcomes") == 0)
    		determinization = DETERMINIZATION_ALL_OUTCOMES_GOAL; // to be changed after the 'while' option loop if reward-based MDP
    	else {
    	  std::cerr << "Unknown determinization. Try `plannerclient --help' for more information."
    	    	<< std::endl;
    	  return -1;
    	}
    	break;
      case 'F':
    	if (strcmp(optarg, "ff") == 0)
    	  deterministic_planner = DETERMINISTIC_PLANNER_FF;
        else if (strcmp(optarg, "mff") == 0)
    	  deterministic_planner = DETERMINISTIC_PLANNER_MFF;
    	else {
    	  std::cerr << "Unknown deterministic planner. Try `plannerclient --help' for more information."
    	    	<< std::endl;
    	  return -1;
    	}
    	break;
      case 't':
    	planning_time = (optarg != 0) ? atol(optarg) : 60000;
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
        std::cerr << "Try `plannerclient --help' for more information."
                  << std::endl;
        return -1;
      }
    }

    if (((alg == ALGORITHM_RDRD) || (alg == ALGORITHM_RDRO)) && (determinization == DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL)) // because of the 'while' option loop in case of reward-based MDP
        determinization = DETERMINIZATION_MOST_PROBABLE_OUTCOME_REWARD;

    if (((alg == ALGORITHM_RDRD) || (alg == ALGORITHM_RDRO)) && (determinization == DETERMINIZATION_ALL_OUTCOMES_GOAL)) // because of the 'while' option loop in case of reward-based MDP
        determinization = DETERMINIZATION_ALL_OUTCOMES_REWARD;

    if ((alg == ALGORITHM_RDRD) && optimized_reward_determinization) // because of the 'while' option loop in case of reward-based MDP
        alg = ALGORITHM_RDRD_STAR;

    if ((alg == ALGORITHM_RDRO) && optimized_reward_determinization) // because of the 'while' option loop in case of reward-based MDP
        alg = ALGORITHM_RDRO_STAR;

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

    std::map<std::string, int> stateVariableIndices;
    std::vector<std::vector<std::string> > stateVariableValues;
    int socket;
    if (ippc)
    {
        std::cout << "Option IPPC Activee. host: " << host << ':' << port << std::endl;
        // Gets RDDL parameters
        //Parser parser(problemFileName);
        //parser.parseTask(stateVariableIndices, stateVariableValues);
    }
    else {
        socket = connect(host.c_str(), port);
        if (socket <= 0) {
            std::cerr << "Could not connect to " << host << ':' << port << std::endl;
            close(socket);
            return 1;
        }
    }

    for (Problem::ProblemMap::const_iterator pi = Problem::begin(); pi != Problem::end(); pi++) {
        current_problem_hack_ = pi->second;
        switch (pln) {

        case PLANNER_COMPLETE :
        {
            if (ippc)
            {
                RddlPlanner p(*(*pi).second, alg, heur, enc, determinization, deterministic_planner, eps, df, probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit, deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials);
                // Create connector to rddlsim and run
                IPPCClient* client = new IPPCClient(&p, host, port, stateVariableIndices, stateVariableValues);

                client->run( (*(*pi).second).name());

                // cout << "RFF complete running time: " << totalTime << endl;
                return 0;
            }
            else
            {
                CompletePlanner p(*(*pi).second, alg, heur, enc, determinization, deterministic_planner, eps, df, probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit, deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials);
                XMLClient xml_client(p, *(*pi).second, client_name, socket);
            }
            break;
        }

        case PLANNER_ANYTIME :
        {
            std::cout << "!!!!! WARNING !!!!!" << std::endl;
            std::cout << "YOU SHOULD NOT USE THE ANYTIME PLANNER: IT DOES NOT WORK PROPERLY!\nUSE THE COMPLETE PLANNER INSTEAD." << std::endl;
            AnytimePlanner p(*(*pi).second, alg, heur, enc, determinization, deterministic_planner, planning_time, eps, df, probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit, deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials);
            XMLClient xml_client(p, *(*pi).second, client_name, socket);
            break;
        }
        }
    }
    close(socket);
  } catch (const BaseException& e) {
    std::cerr << std::endl << client_name << ": " << e.what() << std::endl;
    return 1;
  } catch (const std::exception& e) {
    std::cerr << std::endl << client_name << ": " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cerr << client_name << ": fatal error" << std::endl;
    return -1;
  }

  Problem::clear();
  Domain::clear();

  return 0;
}
