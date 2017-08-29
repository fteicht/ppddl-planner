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

#include "raw_planner.h"
#include "algorithms/base_algorithm.h"
#include "algorithms/graph_gco.h"
#include <mdpsim/client.h>

std::string current_file; // needed to parse PDDL files whose parser declares this variable as extern
int warning_level = 1; // needed to parse PDDL files whose parser declares this variable as extern
int verbosity = 0; // needed to solve problems where some planners declare this variable as extern
const Problem* current_problem_hack_ = NULL; // needed to solve problems where some planners declare this variable as extern

// CLASS RawPlanner

RawPlanner::RawPlanner(const Problem& problem, algorithm_t alg, heuristic_t heur, encoding_t enc,
		determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
		double epsilon, double discount_factor, long time_threshold,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth, unsigned int goals_limit,
		double deterministic_probability_weight, double deterministic_reward_weight,
        unsigned int deterministic_planner_timeout, unsigned int deterministic_planner_max_trials,
		double proportion_updated_states, double minimum_mean, double maximum_variance,
		bool compute_goal_statistics, algorithm_t goal_statistics_algorithm,
        int warn_level, int verb_level)
: BasePlanner(problem, alg, heur, enc, determinization_type, deterministic_planner_type, epsilon, discount_factor,
	probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
	deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
	proportion_updated_states, minimum_mean, maximum_variance),
	domain_(&problem.domain()), problem_(&problem),
    solving_time_(0), resolving_time_(0), nb_of_replannings_(0),
	compute_goal_statistics_(compute_goal_statistics), goal_statistics_algorithm_(goal_statistics_algorithm)
{
	time_threshold_ = time_threshold;
    warning_level = warn_level;
    verbosity = verb_level;
}


RawPlanner::RawPlanner(const std::string& domain, const std::string& problem, algorithm_t alg, heuristic_t heur, encoding_t enc,
		determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
		double epsilon, double discount_factor, long time_threshold,
		double probabilistic_threshold, unsigned int nb_of_particles, unsigned int trajectories_max_depth, unsigned int goals_limit,
		double deterministic_probability_weight, double deterministic_reward_weight,
        unsigned int deterministic_planner_timeout, unsigned int deterministic_planner_max_trials,
		double proportion_updated_states, double minimum_mean, double maximum_variance,
		bool compute_goal_statistics, algorithm_t goal_statistics_algorithm,
        int warn_level, int verb_level)
try : BasePlanner(*parse_domain_and_problem(domain, problem).second, alg, heur, enc, determinization_type, deterministic_planner_type, epsilon, discount_factor,
	probabilistic_threshold, nb_of_particles, trajectories_max_depth, goals_limit,
	deterministic_probability_weight, deterministic_reward_weight, deterministic_planner_timeout, deterministic_planner_max_trials,
	proportion_updated_states, minimum_mean, maximum_variance),
    solving_time_(0), resolving_time_(0), nb_of_replannings_(0),
	compute_goal_statistics_(compute_goal_statistics), goal_statistics_algorithm_(goal_statistics_algorithm)
{
	time_threshold_ = time_threshold;
    warning_level = warn_level;
    verbosity = verb_level;
}
catch(const BaseException& e)
{
    
}


RawPlanner::~RawPlanner()
{
}


std::pair<const Domain*, const Problem*> RawPlanner::parse_domain_and_problem(const std::string& domain, const std::string& problem)
{
    /* The parse function. */
    extern int yyparse();
    /* File to parse. */
    extern FILE* yyin;
    
    // DOMAIN
    
    yyin = fopen(domain.c_str(), "r");
    if (yyin == 0)
        throw BaseException(std::string("Parsing domain ") + domain + ": " + strerror(errno), "RawPlanner::parse_domain");
   
    current_file = domain;
    bool success = (yyparse() == 0);
    fclose(yyin);
    
    if (!success)
        throw BaseException(std::string("Unable to parse domain ") + domain, "RawPlanner::parse_domain");
    
    if (Domain::begin() == Domain::end())
        throw BaseException(std::string("Parsed more than 1 domain in file ") + domain, "RawPlanner::parse_domain");
    
    domain_ = Domain::begin()->second;
    
    // PROBLEM
    
    yyin = fopen(problem.c_str(), "r");
    if (yyin == 0)
        throw BaseException(std::string("Parsing problem ") + problem + ": " + strerror(errno), "RawPlanner::parse_domain");
   
    current_file = problem;
    success = (yyparse() == 0);
    fclose(yyin);
    
    if (!success)
        throw BaseException(std::string("Unable to parse problem ") + problem, "RawPlanner::parse_domain");
    
    if (Problem::begin() == Problem::end())
        throw BaseException(std::string("Parsed more than 1 problem in file ") + problem, "RawPlanner::parse_domain");
    
    problem_ = Problem::begin()->second;
    current_problem_hack_ = problem_;
    
    // RETURN
    
    return std::make_pair(domain_, problem_);
}


const Action& RawPlanner::action(const PddlState& st)
{
	try
	{
		check_solve(st);
		return (algorithm_->get_mdp().get_policy(st));
	}
	catch (TimeoutException& error)
	{
		error.push_function_backtrace("TestPlanner::action");
		throw;
	}
	catch (const BaseException& error)
	{
		throw;
	}
}


std::string RawPlanner::action(const std::string& st)
{
    try
    {
        AtomSet atoms;
        ValueMap values;
        XMLText xmlst(st);
        XMLClient::getState(atoms, values, *problem_, &xmlst);
        PddlState sst(atoms, values);
        std::ostringstream oss;
        XMLClient::sendAction(oss, &action(sst));
        return oss.str();
    }
    catch (const BaseException& error)
    {
        throw;
    }
}


#ifdef HAVE_BOOST_PYTHON
boost::python::str RawPlanner::action(const boost::python::str& st)
{
    try
    {
        return boost::python::str(action(std::string(boost::python::extract<std::string>(st))));
    }
    catch (const BaseException& error)
    {
        throw;
    }
}
#endif


#ifdef HAVE_BOOST_PYTHON
boost::python::dict RawPlanner::action(const boost::python::dict& st)
{
    try
    {
        AtomSet atoms;
        ValueMap values;
        decode_state(st, atoms, values);
        PddlState sst(atoms, values);
        boost::python::dict at;
        encode_action(action(sst), at);
        return at;
    }
    catch (const boost::python::error_already_set& e)
    {
        throw BaseException("The state dictionary is ill-formed (check the type of objects)", "RawPlanner::action");
    }
    catch (const BaseException& e)
    {
        throw;
    }
}
#endif


double RawPlanner::value(const PddlState& st)
{
	try
	{
		check_solve(st);
		return (algorithm_->get_mdp().get_value(st));
	}
	catch (TimeoutException& error)
	{
		error.push_function_backtrace("TestPlanner::value");
		throw;
	}
	catch (const BaseException& error)
	{
		throw;
	}
}


double RawPlanner::value(const std::string& st)
{
    try
    {
        AtomSet atoms;
        ValueMap values;
        XMLText xmlst(st);
        XMLClient::getState(atoms, values, *problem_, &xmlst);
        PddlState sst(atoms, values);
        return value(sst);
    }
    catch (const BaseException& error)
    {
        throw;
    }
}


#ifdef HAVE_BOOST_PYTHON
double RawPlanner::value(const boost::python::str& st)
{
    try
    {
        return value(std::string(boost::python::extract<std::string>(st)));
    }
    catch (const BaseException& error)
    {
        throw;
    }
}
#endif


#ifdef HAVE_BOOST_PYTHON
double RawPlanner::value(const boost::python::dict& st)
{
    try
    {
        AtomSet atoms;
        ValueMap values;
        decode_state(st, atoms, values);
        PddlState sst(atoms, values);
        return value(sst);
    }
    catch (const boost::python::error_already_set& e)
    {
        throw BaseException("The state dictionary is ill-formed (check the type of objects)", "RawPlanner::action");
    }
    catch (const BaseException& e)
    {
        throw;
    }
}
#endif


void RawPlanner::check_solve(const PddlState& st)
{
	if (!(algorithm_->policy_defined(st)))
	{
		long resolving_time_begin = get_time_milli();
        nb_of_replannings_++;
        
		std::cout << "Solving from state " << st << std::endl;
		algorithm_->solve_initialize(st);
		bool alg_converged = false;

		while ((get_time_milli() - resolving_time_begin) <= time_threshold_)
		{
			if (!ram_available())
                std::cout << "Giving up by lack of sufficient RAM!" << std::endl;
				break;

			algorithm_->solve_progress();

			if (algorithm_->has_converged())
			{
				alg_converged = true;
				break;
			}
		}

		if (!alg_converged)
			throw TimeoutException("algorithm has not yet converged", "CompleteTestPlanner::check_solve");

		resolving_time_ = (get_time_milli() - resolving_time_begin);
		solving_time_ += resolving_time_;

        if (compute_goal_statistics_)
        {
            std::cout << "Computing goal statistics..." << std::endl;
            std::pair<double, double> res = algorithm_->compute_goal_statistics(goal_statistics_algorithm_, st);
            std::cout << "   goal probability: " << res.first << std::endl;
            std::cout << "   goal cost: " << res.second << std::endl;
        }
	}
}


#ifdef HAVE_BOOST_PYTHON
void RawPlanner::decode_state(const boost::python::dict& state, AtomSet& atoms, ValueMap& values)
{
    atoms.clear();
    values.clear();
    
    // ATOMS
    
    if (!state.has_key("atoms"))
        throw BaseException("State has no 'atoms' key", "RawPlanner::get_state");
        
    boost::python::list al = boost::python::extract<boost::python::list>(state.get("atoms"));
    
    for (boost::python::stl_input_iterator<boost::python::dict> i(al); i != boost::python::stl_input_iterator<boost::python::dict>(); i++)
    {
        boost::python::dict atom = *i;
        
        if (!atom.has_key("predicate"))
            throw BaseException ("State atom has no 'predicate' key", "RawPlanner::get_state");
        
        boost::python::str predicate = boost::python::extract<boost::python::str>(atom.get("predicate"));
        const Predicate* p = domain_->predicates().find_predicate(boost::python::extract<std::string>(predicate));
        if (!p)
            throw BaseException(std::string("State predicate ") + std::string(boost::python::extract<std::string>(predicate)) + " is unknown to the domain", "RawPlanner::get_state");
        
        boost::python::list terms = boost::python::extract<boost::python::list>(atom.get("terms"));
        
        if (boost::python::len(terms) != PredicateTable::parameters(*p).size())
            throw BaseException(std::string("State predicate ") + std::string(boost::python::extract<std::string>(predicate)) + "has not the correct number of parameters", "RawPlanner::get_state");
        
        TermList t;
        size_t argIndex = 0;
        
        for (boost::python::stl_input_iterator<boost::python::str> j(terms); j != boost::python::stl_input_iterator<boost::python::str>(); j++)
        {
            Type correctType = PredicateTable::parameters(*p)[argIndex];
            argIndex++;
            const Object* o = problem_->terms().find_object(boost::python::extract<std::string>(*j));
            
            if (o)
            {
                if (!TypeTable::subtype(TermTable::type(*o), correctType))
                {
                    std::ostringstream oss;
                    oss << "Parameter " << argIndex << " of state predicate " << std::string(boost::python::extract<std::string>(predicate)) << " is of incorrect type";
                    throw BaseException(oss.str(), "RawPlanner::get_state");
                }
            }
            else
            {
                o = domain_->terms().find_object(boost::python::extract<std::string>(*j));
                
                if (!o)
                {
                    std::ostringstream oss;
                    oss << "Parameter " << argIndex << " of state predicate " << std::string(boost::python::extract<std::string>(predicate)) << " is unknown to the domain nor the problem";
                    throw BaseException(oss.str(), "RawPlanner::get_state");
                }
                if (!TypeTable::subtype(TermTable::type(*o), correctType))
                {
                    std::ostringstream oss;
                    oss << "Parameter " << argIndex << " of state predicate " << std::string(boost::python::extract<std::string>(predicate)) << " is of incorrect type";
                    throw BaseException(oss.str(), "RawPlanner::get_state");
                }
            }
            
            t.push_back(*o);
        }
        
        const Atom* a = &Atom::make(*p, t);
        RCObject::ref(a);
        atoms.insert(a);
    }
    
    // FLUENTS
    
    if (!state.has_key("fluents"))
        throw BaseException("State has no 'fluents' key", "RawPlanner::get_state");
        
    boost::python::list fl = boost::python::extract<boost::python::list>(state.get("fluents"));
    
    
    for (boost::python::stl_input_iterator<boost::python::tuple> i(fl); i != boost::python::stl_input_iterator<boost::python::tuple>(); i++)
    {
        boost::python::tuple fluent = *i;
        boost::python::dict fd = boost::python::extract<boost::python::dict>(fluent[0]);
        double fv = boost::python::extract<double>(fluent[1]);
        
        if (!fd.has_key("function"))
            throw BaseException ("State fluent has no 'function' key", "RawPlanner::get_state");
        
        boost::python::str function = boost::python::extract<boost::python::str>(fd.get("function"));
        const Function* f = domain_->functions().find_function(boost::python::extract<std::string>(function));
        if (!f)
            throw BaseException(std::string("State function ") + std::string(boost::python::extract<std::string>(function)) + " is unknown to the domain", "RawPlanner::get_state");
        
        boost::python::list terms = boost::python::extract<boost::python::list>(fd.get("terms"));
        
        if (boost::python::len(terms) != FunctionTable::parameters(*f).size())
            throw BaseException(std::string("State function ") + std::string(boost::python::extract<std::string>(function)) + "has not the correct number of parameters", "RawPlanner::get_state");
        
        TermList t;
        size_t argIndex = 0;
        
        for (boost::python::stl_input_iterator<boost::python::str> j(terms); j != boost::python::stl_input_iterator<boost::python::str>(); j++)
        {
            Type correctType = FunctionTable::parameters(*f)[argIndex];
            argIndex++;
            const Object* o = problem_->terms().find_object(boost::python::extract<std::string>(*j));
            
            if (o)
            {
                if (!TypeTable::subtype(TermTable::type(*o), correctType))
                {
                    std::ostringstream oss;
                    oss << "Parameter " << argIndex << " of state function " << std::string(boost::python::extract<std::string>(function)) << " is of incorrect type";
                    throw BaseException(oss.str(), "RawPlanner::get_state");
                }
            }
            else
            {
                o = domain_->terms().find_object(boost::python::extract<std::string>(*j));
                
                if (!o)
                {
                    std::ostringstream oss;
                    oss << "Parameter " << argIndex << " of state function " << std::string(boost::python::extract<std::string>(function)) << " is unknown to the domain nor the problem";
                    throw BaseException(oss.str(), "RawPlanner::get_state");
                }
                if (!TypeTable::subtype(TermTable::type(*o), correctType))
                {
                    std::ostringstream oss;
                    oss << "Parameter " << argIndex << " of state function " << std::string(boost::python::extract<std::string>(function)) << " is of incorrect type";
                    throw BaseException(oss.str(), "RawPlanner::get_state");
                }
            }
            
            t.push_back(*o);
        }
        
        const Fluent* ff = &Fluent::make(*f, t);
        RCObject::ref(ff);
        values.insert(std::make_pair(ff, fv));
    }
}
#endif


#ifdef HAVE_BOOST_PYTHON
void RawPlanner::encode_action(const Action& action, boost::python::dict& result)
{
    result = boost::python::dict();
    result[boost::python::str("name")] = boost::python::str(action.name());
    boost::python::list terms;
    
    for (ObjectList::const_iterator oi = action.arguments().begin(); oi != action.arguments().end(); oi++)
    {
        std::ostringstream oss;
        oss << (*oi);
        terms.append(boost::python::str(oss.str()));
    }
    
    result[boost::python::str("terms")] = terms;
}
#endif
