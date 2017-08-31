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
	initialize(time_threshold, warn_level, verb_level);
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
	initialize(time_threshold, warn_level, verb_level);
}
catch(const BaseException& e)
{
    std::cerr << "Unable to construct the planner! " << e.what() << std::endl;
}


RawPlanner::~RawPlanner()
{
}


void RawPlanner::initialize(long time_threshold, int warn_level, int verb_level)
{
    time_threshold_ = time_threshold;
    warning_level = warn_level;
    verbosity = verb_level;
}


#ifdef HAVE_BOOST_PYTHON
boost::python::object RawPlannerInit(boost::python::tuple args, boost::python::dict kwargs)
{
    
}
#endif


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
    
    if (verbosity > 1)
        std::cout << "DOMAIN:" << std::endl << *domain_ << std::endl;
    
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
    
    if (verbosity > 1)
        std::cout << "PROBLEM:" << std::endl << *problem_ << std::endl;
    
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
		error.push_function_backtrace("RawPlanner::action");
		throw;
	}
	catch (BaseException& error)
	{
        error.push_function_backtrace("RawPlanner::action");
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
    catch (BaseException& e)
    {
        e.push_function_backtrace("RawPlanner::action");
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
    catch (BaseException& e)
    {
        e.push_function_backtrace("RawPlanner::action");
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
    catch (BaseException& e)
    {
        e.push_function_backtrace("RawPlanner::action");
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
		error.push_function_backtrace("RawPlanner::value");
		throw;
	}
	catch (BaseException& error)
	{
        error.push_function_backtrace("RawPlanner::value");
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
    catch (BaseException& error)
    {
        error.push_function_backtrace("RawPlanner::value");
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
    catch (BaseException& error)
    {
        error.push_function_backtrace("RawPlanner::value");
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
    catch (BaseException& e)
    {
        e.push_function_backtrace("RawPlanner::value");
        throw;
    }
}
#endif


void RawPlanner::init(PddlState& state, bool& goal)
{
    try
    {
        state._atoms_() = problem_->init_atoms();
        state._values_() = problem_->init_values();
        
        for (EffectList::const_iterator ei = problem_->init_effects().begin(); ei != problem_->init_effects().end(); ei++)
        {
            AtomList adds;
            AtomList deletes;
            UpdateList updates;
            (*ei)->state_change(adds, deletes, updates, problem_->terms(), state.atoms(), state.values());
            state._atoms_().insert(adds.begin(), adds.end());
            
            for (UpdateList::const_iterator ui = updates.begin(); ui != updates.end(); ui++)
            {
                (*ui)->affect(state._values_());
            }
        }
        
        goal = problem_->goal().holds(problem_->terms(), state.atoms(), state.values());
        
        if (goal)
        {
            const Fluent& goal_achieved_fluent = Fluent::make(domain_->goal_achieved(), TermList());
            state._values_()[&goal_achieved_fluent] = 1;
            
            if (problem_->goal_reward() != 0)
            {
                problem_->goal_reward()->affect(state._values_());
            }
        }
    }
    catch (BaseException& error)
    {
        error.push_function_backtrace("RawPlanner::init");
        throw;
    }
}


#ifdef HAVE_BOOST_PYTHON
boost::python::tuple RawPlanner::init()
{
    try
    {
        PddlState initst;
        bool goal;
        init(initst, goal);
        boost::python::dict st;
        encode_state(initst.atoms(), initst.values(), st);
        return boost::python::make_tuple(st, goal);
    }
    catch (BaseException& error)
    {
        error.push_function_backtrace("RawPlanner::init");
        throw;
    }
}
#endif


void RawPlanner::next(const PddlState& income, const Action& action, PddlState& outcome, double& reward, bool& goal)
{
    try
    {
        algorithm_->get_transitions_blackbox().random_transition(income, action, outcome, reward, goal);
    }
    catch (BaseException& e)
    {
        e.push_function_backtrace("RawPlanner::next");
        throw;
    }
}


/*void RawPlanner::next(const std::string& income, const std::string& action, std::string& outcome, double& reward, bool& goal)
{
    AtomSet atoms;
    ValueMap values;
    XMLText xmlst(income);
    XMLClient::getState(atoms, values, *problem_, &xmlst);
    PddlState inst(atoms, values);
}*/


#ifdef HAVE_BOOST_PYTHON
boost::python::tuple RawPlanner::next(const boost::python::dict& income, const boost::python::dict& action)
{
    try
    {
        AtomSet atoms;
        ValueMap values;
        decode_state(income, atoms, values);
        PddlState inst(atoms, values);
        const Action* act;
        decode_action(action, act);
        PddlState outst;
        double reward;
        bool goal;
        next(inst, *act, outst, reward, goal);
        boost::python::dict outcome;
        encode_state(outst.atoms(), outst.values(), outcome);
        return boost::python::make_tuple(outcome, reward, goal);
    }
    catch (BaseException& e)
    {
        e.push_function_backtrace("RawPlanner::next");
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
            {
                std::cout << "Giving up by lack of sufficient RAM!" << std::endl;
				break;
            }

			algorithm_->solve_progress();

			if (algorithm_->has_converged())
			{
				alg_converged = true;
				break;
			}
		}

		if (!alg_converged)
			throw TimeoutException("algorithm has not yet converged", "RawPlanner::check_solve");

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
        throw BaseException("State has no 'atoms' key", "RawPlanner::decode_state");
        
    boost::python::list al = boost::python::extract<boost::python::list>(state.get("atoms"));
    
    for (boost::python::stl_input_iterator<boost::python::dict> i(al); i != boost::python::stl_input_iterator<boost::python::dict>(); i++)
    {
        boost::python::dict atom = *i;
        
        if (!atom.has_key("predicate"))
            throw BaseException ("State atom has no 'predicate' key", "RawPlanner::decode_state");
        
        boost::python::str predicate = boost::python::extract<boost::python::str>(atom.get("predicate"));
        const Predicate* p = domain_->predicates().find_predicate(boost::python::extract<std::string>(predicate));
        
        if (!p)
            throw BaseException(std::string("State predicate ") + std::string(boost::python::extract<std::string>(predicate)) + " is unknown to the domain", "RawPlanner::decode_state");
        
        if (!atom.has_key("terms"))
            throw BaseException ("State atom has no 'terms' key", "RawPlanner::decode_state");
        
        boost::python::list terms = boost::python::extract<boost::python::list>(atom.get("terms"));
        
        if (boost::python::len(terms) != PredicateTable::parameters(*p).size())
            throw BaseException(std::string("State predicate ") + std::string(boost::python::extract<std::string>(predicate)) + "has not the correct number of parameters", "RawPlanner::decode_state");
        
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
                    throw BaseException(oss.str(), "RawPlanner::decode_state");
                }
            }
            else
            {
                o = domain_->terms().find_object(boost::python::extract<std::string>(*j));
                
                if (!o)
                {
                    std::ostringstream oss;
                    oss << "Parameter " << argIndex << " of state predicate " << std::string(boost::python::extract<std::string>(predicate)) << " is unknown to the domain nor the problem";
                    throw BaseException(oss.str(), "RawPlanner::decode_state");
                }
                if (!TypeTable::subtype(TermTable::type(*o), correctType))
                {
                    std::ostringstream oss;
                    oss << "Parameter " << argIndex << " of state predicate " << std::string(boost::python::extract<std::string>(predicate)) << " is of incorrect type";
                    throw BaseException(oss.str(), "RawPlanner::decode_state");
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
        throw BaseException("State has no 'fluents' key", "RawPlanner::decode_state");
        
    boost::python::list fl = boost::python::extract<boost::python::list>(state.get("fluents"));
    
    for (boost::python::stl_input_iterator<boost::python::tuple> i(fl); i != boost::python::stl_input_iterator<boost::python::tuple>(); i++)
    {
        boost::python::tuple fluent = *i;
        boost::python::dict fd = boost::python::extract<boost::python::dict>(fluent[0]);
        double fv = boost::python::extract<double>(fluent[1]);
        
        if (!fd.has_key("function"))
            throw BaseException ("State fluent has no 'function' key", "RawPlanner::decode_state");
        
        boost::python::str function = boost::python::extract<boost::python::str>(fd.get("function"));
        const Function* f = domain_->functions().find_function(boost::python::extract<std::string>(function));
        
        if (!f)
            throw BaseException(std::string("State function ") + std::string(boost::python::extract<std::string>(function)) + " is unknown to the domain", "RawPlanner::decode_state");
        
        if (!fd.has_key("terms"))
            throw BaseException ("State fluent has no 'terms' key", "RawPlanner::decode_state");
        
        boost::python::list terms = boost::python::extract<boost::python::list>(fd.get("terms"));
        
        if (boost::python::len(terms) != FunctionTable::parameters(*f).size())
            throw BaseException(std::string("State function ") + std::string(boost::python::extract<std::string>(function)) + "has not the correct number of parameters", "RawPlanner::decode_state");
        
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
                    throw BaseException(oss.str(), "RawPlanner::decode_state");
                }
            }
            else
            {
                o = domain_->terms().find_object(boost::python::extract<std::string>(*j));
                
                if (!o)
                {
                    std::ostringstream oss;
                    oss << "Parameter " << argIndex << " of state function " << std::string(boost::python::extract<std::string>(function)) << " is unknown to the domain nor the problem";
                    throw BaseException(oss.str(), "RawPlanner::decode_state");
                }
                if (!TypeTable::subtype(TermTable::type(*o), correctType))
                {
                    std::ostringstream oss;
                    oss << "Parameter " << argIndex << " of state function " << std::string(boost::python::extract<std::string>(function)) << " is of incorrect type";
                    throw BaseException(oss.str(), "RawPlanner::decode_state");
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
void RawPlanner::encode_state(const AtomSet& atoms, const ValueMap& values, boost::python::dict& result)
{
    result = boost::python::dict();
    boost::python::list atom_list;
    
    for (AtomSet::const_iterator a = atoms.begin() ; a != atoms.end() ; ++a)
    {
        boost::python::dict atom_dict;
        atom_dict[boost::python::str("predicate")] = boost::python::str(domain_->predicates().name((*a)->predicate()));
        boost::python::list term_list;
        
        for (TermList::const_iterator t = (*a)->terms().begin() ; t != (*a)->terms().end() ; ++t)
        {
            std::ostringstream oss;
            oss << (*t);
            
            if (!(t->object()))
                throw BaseException(std::string("Atom term ") + oss.str() + " is not an object", "RawPlanner::encode_state");
            
            
            term_list.append(boost::python::str(oss.str()));
        }
        
        atom_dict[boost::python::str("terms")] = term_list;
        atom_list.append(atom_dict);
    }
    
    result[boost::python::str("atoms")] = atom_list;
    boost::python::list fluent_list;
    
    for (ValueMap::const_iterator f = values.begin() ; f != values.end() ; ++f)
    {
        boost::python::dict fluent_dict;
        fluent_dict[boost::python::str("function")] = boost::python::str(domain_->functions().name(f->first->function()));
        boost::python::list term_list;
        
        for (TermList::const_iterator t = f->first->terms().begin() ; t != f->first->terms().end() ; ++t)
        {
            std::ostringstream oss;
            oss << (*t);
            
            if (!(t->object()))
                throw BaseException(std::string("Fluent term ") + oss.str() + " is not an object", "RawPlanner::encode_state");
            
            
            term_list.append(boost::python::str(oss.str()));
        }
        
        fluent_dict[boost::python::str("terms")] = term_list;
        boost::python::tuple fluent_tuple = boost::python::make_tuple(fluent_dict, f->second.double_value());
        fluent_list.append(fluent_tuple);
    }
    
    result[boost::python::str("fluents")] = fluent_list;
}
#endif


#ifdef HAVE_BOOST_PYTHON
void RawPlanner::decode_action(const boost::python::dict& action, const Action*& result)
{
    if (!action.has_key("name"))
        throw BaseException("Action has no 'name' key", "RawPlanner::decode_action");
        
    boost::python::str name = boost::python::extract<boost::python::str>(action.get("name"));
    const ActionSchema* as = domain_->find_action(boost::python::extract<std::string>(name));
    
    if (!as)
        throw BaseException(std::string("action ") + std::string(boost::python::extract<std::string>(name)) + " is unknown to the domain", "RawPlanner::decode_action");
    
    if (!action.has_key("terms"))
        throw BaseException ("Action has no 'terms' key", "RawPlanner::decode_action");
    
    boost::python::list terms = boost::python::extract<boost::python::list>(action.get("terms"));
    
    if (boost::python::len(terms) != as->parameters().size())
        throw BaseException(std::string("Action ") + std::string(boost::python::extract<std::string>(name)) + "has not the correct number of parameters", "RawPlanner::decode_action");
    
    std::string actname = boost::python::extract<std::string>(name);
    Action act(actname);
    size_t argIndex = 0;
    
    for (boost::python::stl_input_iterator<boost::python::str> j(terms); j != boost::python::stl_input_iterator<boost::python::str>(); j++)
    {
        Type correctType = TermTable::type(Term(as->parameters()[argIndex]));
        argIndex++;
        const Object* o = problem_->terms().find_object(boost::python::extract<std::string>(*j));
        
        if (o)
        {
            if (!TypeTable::subtype(TermTable::type(*o), correctType))
            {
                std::ostringstream oss;
                oss << "Parameter " << argIndex << " of action " << std::string(boost::python::extract<std::string>(name)) << " is of incorrect type";
                throw BaseException(oss.str(), "RawPlanner::decode_action");
            }
        }
        else
        {
            o = domain_->terms().find_object(boost::python::extract<std::string>(*j));
            
            if (!o)
            {
                std::ostringstream oss;
                oss << "Parameter " << argIndex << " of action " << std::string(boost::python::extract<std::string>(name)) << " is unknown to the domain nor the problem";
                throw BaseException(oss.str(), "RawPlanner::decode_action");
            }
            if (!TypeTable::subtype(TermTable::type(*o), correctType))
            {
                std::ostringstream oss;
                oss << "Parameter " << argIndex << " of action " << std::string(boost::python::extract<std::string>(name)) << " is of incorrect type";
                throw BaseException(oss.str(), "RawPlanner::decode_action");
            }
        }
        
        act.add_argument(*o);
    }
    
    ActionSet::const_iterator ai = problem_->actions().find(&act);
    
    if (ai == problem_->actions().end())
        throw BaseException ("Action is unknown to the problem", "RawPlanner::decode_action");
    
    result = *ai;
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


#ifdef HAVE_BOOST_PYTHON
boost::python::str (RawPlanner::*action_str)(const boost::python::str&) = &RawPlanner::action;
boost::python::dict (RawPlanner::*action_dict)(const boost::python::dict&) = &RawPlanner::action;
double (RawPlanner::*value_str)(const boost::python::str&) = &RawPlanner::value;
double (RawPlanner::*value_dict)(const boost::python::dict&) = &RawPlanner::value;
boost::python::tuple (RawPlanner::*init_dict)() = &RawPlanner::init;
boost::python::tuple (RawPlanner::*next_dict)(const boost::python::dict&, const boost::python::dict&) = &RawPlanner::next;
    
BOOST_PYTHON_MODULE(libppddl_planner)
{
    boost::python::enum_<algorithm_t>("algorithm")
                                          .value("vi", ALGORITHM_VI)
                                          .value("fsp", ALGORITHM_FSP)
                                          .value("fsp-star", ALGORITHM_FSP_STAR)
                                          .value("trfsp_star", ALGORITHM_TRFSP_STAR)
                                          .value("bfdt", ALGORITHM_BFDT)
                                          .value("psp_star", ALGORITHM_PSP_STAR)
                                          .value("fdp", ALGORITHM_FDP)
                                          .value("lao", ALGORITHM_LAO)
                                          .value("ilao", ALGORITHM_IMPROVED_LAO)
                                          .value("rtdp", ALGORITHM_RTDP)
                                          .value("lrtdp", ALGORITHM_LRTDP)
                                          .value("ldfs", ALGORITHM_LDFS)
                                          .value("od", ALGORITHM_OD)
                                          .value("rdpg", ALGORITHM_RDPG)
                                          .value("rdrg", ALGORITHM_RDRG)
                                          .value("rdbg", ALGORITHM_RDBG)
                                          .value("rdrd", ALGORITHM_RDRD)
                                          .value("rdro", ALGORITHM_RDRO)
                                          .value("rdrd_star", ALGORITHM_RDRD_STAR)
                                          .value("rdro_star", ALGORITHM_RDRO_STAR)
                                          .value("gco", ALGORITHM_GCO)
                                          .value("gce", ALGORITHM_GCE)
                                          .value("mvpfpi", ALGORITHM_MVPFPI)
                                          .value("mvhopi", ALGORITHM_MVHOPI)
                                          .value("mvpfas", ALGORITHM_MVPFAS)
                                          .value("mvhoas", ALGORITHM_MVHOAS)
                                          .value("mvpfssp", ALGORITHM_MVPFSSP);
    boost::python::enum_<heuristic_t>("heuristic")
                                          .value("zero", HEURISTIC_ZERO)
                                          .value("pddl_step_distance", HEURISTIC_PDDL_STEP_DISTANCE)
                                          .value("native_step_distance", HEURISTIC_NATIVE_STEP_DISTANCE)
                                          .value("relaxed_step_distance", HEURISTIC_RELAXED_STEP_DISTANCE)
                                          .value("relaxed_rewards", HEURISTIC_RELAXED_REWARDS)
                                          .value("relaxed_planning_graph", HEURISTIC_RELAXED_PLANNING_GRAPH)
                                          .value("bounds", HEURISTIC_BOUNDS)
                                          .value("hadd", HEURISTIC_HADD)
                                          .value("hmax", HEURISTIC_HMAX)
                                          .value("hdet", HEURISTIC_HDET);
    boost::python::enum_<encoding_t>("encoding")
                                         .value("graph", ENCODING_GRAPH)
                                         .value("symbolic", ENCODING_SYMBOLIC);
    boost::python::enum_<determinization_t>("determinization")
                                                .value("most_probable_outcome_goal", DETERMINIZATION_MOST_PROBABLE_OUTCOME_GOAL)
                                                .value("most_probable_outcome_reward", DETERMINIZATION_MOST_PROBABLE_OUTCOME_REWARD)
                                                .value("all_outcomes_goal", DETERMINIZATION_ALL_OUTCOMES_GOAL)
                                                .value("all_outcomes_reward", DETERMINIZATION_ALL_OUTCOMES_REWARD);
    boost::python::enum_<deterministic_planner_t>("deterministic_planner")
                                                      .value("ff", DETERMINISTIC_PLANNER_FF)
                                                      .value("mff", DETERMINISTIC_PLANNER_MFF);
    boost::python::class_<ParameterRawPlanner>("Planner", boost::python::no_init )
                                                   .def("action", action_str, (boost::python::arg("state")))
                                                   .def("action", action_dict, (boost::python::arg("state")))
                                                   .def("value", value_str, (boost::python::arg("state")))
                                                   .def("value", value_dict, (boost::python::arg("state")))
                                                   .def("init", init_dict)
                                                   .def("next", next_dict, (boost::python::arg("state"), boost::python::arg("action")))
                                                   .def("get_resolving_time", &RawPlanner::get_resolving_time)
                                                   .def("get_solving_time", &RawPlanner::get_solving_time)
                                                   .def("get_nb_of_replannings", &RawPlanner::get_nb_of_replannings)
                                                   .def(boost::parameter::python::init<boost::mpl::vector<
                                                       tag::domain(std::string),
                                                       tag::problem(std::string),
                                                       tag::algorithm(algorithm_t),
                                                       tag::heuristic(heuristic_t),
                                                       tag::encoding(encoding_t),
                                                       tag::determinization(determinization_t),
                                                       tag::deterministic_planner(deterministic_planner_t),
                                                       tag::epsilon*(double),
                                                       tag::discount_factor*(double),
                                                       tag::time_threshold*(long),
                                                       tag::probabilistic_threshold*(double),
                                                       tag::nb_of_particles*(unsigned int),
                                                       tag::trajectories_max_depth*(unsigned int),
                                                       tag::goals_limit*(unsigned int),
                                                       tag::deterministic_probability_weight*(double),
                                                       tag::deterministic_reward_weight*(double),
                                                       tag::deterministic_planner_timeout*(unsigned int),
                                                       tag::deterministic_planner_max_trials*(unsigned int),
                                                       tag::proportion_updated_states*(double),//> >());/*,
                                                       tag::minimum_mean*(double),
                                                       tag::maximum_variance*(double),
                                                       tag::compute_goal_statistics*(bool),
                                                       tag::goal_statistics_algorithm*(algorithm_t),
                                                       tag::warning_level*(int),
                                                       tag::verbosity_level*(int)> >());
                                          
                                          /*const std::string&, const std::string&, // C++ constructor, shadowed by raw constructor
                                                              algorithm_t, heuristic_t, encoding_t,
			                                                  determinization_t, deterministic_planner_t,
			                                                  boost::python::optional<double, double, long,
			                                                  double, unsigned int,
                                                              unsigned int, unsigned int,
                                                              double, double,
                                                              unsigned int, unsigned int,
			                                                  double, double, double,
                                                              bool, algorithm_t,
                                                              int, int> >((
                                                                 boost::python::arg("domain"),
                                                                 boost::python::arg("problem"),
                                                                 boost::python::arg("algorithm"),
                                                                 boost::python::arg("heuristic"),
                                                                 boost::python::arg("encoding"),
                                                                 boost::python::arg("determinization"),
                                                                 boost::python::arg("deterministic_planner"),
                                                                 boost::python::arg("epsilon"),
                                                                 boost::python::arg("discount_factor"),
                                                                 boost::python::arg("time_threshold"),
                                                                 boost::python::arg("probabilistic_threshold"),
                                                                 boost::python::arg("nb_of_particles"),
                                                                 boost::python::arg("trajectories_max_depth"),
                                                                 boost::python::arg("goals_limit"),
                                                                 boost::python::arg("deterministic_probability_weight"),
                                                                 boost::python::arg("deterministic_reward_weight"),
                                                                 boost::python::arg("deterministic_planner_timeout"),
                                                                 boost::python::arg("deterministic_planner_max_trials"),
                                                                 boost::python::arg("proportion_updated_states"),
                                                                 boost::python::arg("minimum_mean"),
                                                                 boost::python::arg("maximum_variance"),
                                                                 boost::python::arg("compute_goal_statistics"),
                                                                 boost::python::arg("goal_statistics_algorithm"),
                                                                 boost::python::arg("warning_level"),
                                                                 boost::python::arg("verbosity_level"))));*/
}
#endif
