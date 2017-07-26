/* 
   ppddl-planner - client for IPPC'08

   Copyright (C) 2012 Florent Teichteil-Koenigsbuch and Guillaume Infantes and Ugur Kuter

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
#include <limits>
#include "graph_gco.h"


GraphGCO::GraphGCO(const Problem& pb, double epsilon)
try : BaseAlgorithm(pb, epsilon, 1.0), GoalStatisticsAlgorithm(pb, epsilon),
      GraphAlgorithm<GraphStateWT<GraphStateProbability> >(pb, epsilon, 1.0)
{
    tb_->set_metric(TransitionsBlackbox::METRIC_REWARD);
}
catch (BaseException& error)
{
	error.push_function_backtrace("GraphGCO::GraphGCO");
	throw;
}


double GraphGCO::goal_probability(const PddlState& st) const
{
    GraphMDP<GraphStateWT<GraphStateProbability> >::states_map_t::const_iterator Ist = mdp_->get_states().find(&st);
    
    if (Ist != mdp_->get_states().end())
        return Ist->second->get_probability();
    else
        return 0.0;
}


double GraphGCO::goal_cost(const PddlState& st) const
{
    GraphMDP<GraphStateWT<GraphStateProbability> >::states_map_t::const_iterator Ist = mdp_->get_states().find(&st);
    
    if (Ist != mdp_->get_states().end())
        return Ist->second->get_value();
    else
        return 0.0;
}


bool GraphGCO::policy_defined(const PddlState& st)
{
	return ((mdp_->explored(st)) && (mdp_->get_state(st).policy_defined()));
}


void GraphGCO::solve_initialize(const PddlState& st)
{
	// generate all reachable states
	mdp_->add_state(st);
	GraphMDP<GraphStateWT<GraphStateProbability> >::new_states_list_t new_states;
	new_states.push_back(mdp_->_states_().find(&st));
	
	while (!new_states.empty())
	{
		mdp_->transitions(*(new_states.front()->second));
		new_states.pop_front();
		new_states.insert(new_states.end(), mdp_->new_states().begin(), mdp_->new_states().end());
	}
	
	record_values_.resize(mdp_->get_states().size(), std::make_pair(0.0, 0.0));
	probability_converged_ = false;
    
    for (GraphMDP<GraphStateWT<GraphStateProbability> >::states_set_t::iterator Igs = mdp_->get_goal_states().begin() ; Igs != mdp_->get_goal_states().end() ; ++Igs)
        (*Igs)->set_probability(1.0);
    
    /*for (GraphMDP<GraphStateWT<GraphStateProbability> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
	{
        if (mdp_->goal(*(Ist->second)))
            Ist->second->set_probability(1.0);
        else
            Ist->second->set_probability(0.0);
        
        Ist->second->set_value(0.0);
    }*/
}


void GraphGCO::solve_progress()
{
	/*// improve state values
	std::list<std::pair<double, double> >::iterator Ivl = record_values_.begin();
	bellman_error_ = 0.0;
    
	for (GraphMDP<GraphStateWT<GraphStateProbability> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
	{
        if (mdp_->goal(*(Ist->second)))
        {
            Ivl->first = 1.0;
            Ivl->second = 0.0;
            ++Ivl;
            continue;
        }
        
        std::list<GraphMDP<GraphStateWT<GraphStateProbability> >::actions_map_t::const_iterator> aset;
        double max_prob = -1.0;
        
        for (GraphMDP<GraphStateWT<GraphStateProbability> >::actions_map_t::const_iterator Iac = Ist->second->get_transitions().begin() ; Iac != Ist->second->get_transitions().end() ; ++Iac)
        {
            double aprob = 0.0;
            
            for (GraphMDP<GraphStateWT<GraphStateProbability> >::transition_list_t::const_iterator Ieff = Iac->second.begin() ; Ieff != Iac->second.end() ; ++Ieff)
                aprob += ((*Ieff)->get_probability()) * ((*Ieff)->get_outcome().get_probability());
            
            //if (aprob > (Ist->second->get_probability()))
            //{
                if (aprob > max_prob)
                {
                    aset.clear();
                    aset.push_back(Iac);
                    max_prob = aprob;
                }
                else if (!(aprob < max_prob))
                    aset.push_back(Iac);
            //}
        }
        
        //max_prob = std::max(max_prob, Ist->second->get_probability());
        Ivl->first = max_prob;
        
        if (max_prob > 0.0)
        {//std::cout << *(Ist->first) << std::endl; std::cout << "mp=" << max_prob << std::endl;
            double max_reward = - std::numeric_limits<double>::max();

            for (std::list<GraphMDP<GraphStateWT<GraphStateProbability> >::actions_map_t::const_iterator>::const_iterator Iac = aset.begin() ; Iac != aset.end() ; ++Iac)
            {//std::cout << *((*Iac)->first) << std::endl;
                double areward = 0.0;

                for (GraphMDP<GraphStateWT<GraphStateProbability> >::transition_list_t::const_iterator Ieff = (*Iac)->second.begin() ; Ieff != (*Iac)->second.end() ; ++Ieff)
                    areward += (((*Ieff)->get_probability()) * ((*Ieff)->get_outcome().get_probability())) * (((*Ieff)->get_reward()) + ((*Ieff)->get_outcome().get_value()));

                areward /= max_prob;
                
                if (areward > max_reward)
                {
                    max_reward = areward;
                    
                    //try{
                      //  if (&(Ist->second->get_policy()) != ((*Iac)->first)) std::cout << "changed" << std::endl;} catch (...) {}
                    Ist->second->set_policy(*((*Iac)->first));
                }
            }
            
            Ivl->second = max_reward;//std::cout << "mr=" << max_reward << std::endl;
        }
        else
        {
            Ist->second->set_policy(*(Ist->second->get_transitions().begin()->first));
            Ivl->second = 0.0;
        }
        
        //if (&(Ist->second->get_policy()) != amap[Ist->second] && amap[Ist->second] != NULL) {
        //    std::cout << "different: " << *(Ist->first) << std::endl << "     " << *(amap[Ist->second]) << " -> " << (Ist->second->get_policy()) << std::endl;
        //}
        amap[Ist->second] = &(Ist->second->get_policy());
        
        double mval = std::max(std::fabs((Ist->second->get_probability()) - (Ivl->first)),
                               std::fabs((Ist->second->get_value()) - (Ivl->second)));
		bellman_error_ = std::max(bellman_error_, mval);
        ++Ivl;
	}
    //std::cout << "be=" << be << std::endl;
    
    // record values
	Ivl = record_values_.begin();
	
	for (GraphMDP<GraphStateWT<GraphStateProbability> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
	{
        Ist->second->set_probability(Ivl->first);
		Ist->second->set_value(Ivl->second);
		++Ivl;
	}*/
    
    bellman_error_ = 0.0;
    std::list<std::pair<double, double> >::iterator Ivl = record_values_.begin();
    
    if (!probability_converged_)
    {
        for (GraphMDP<GraphStateWT<GraphStateProbability> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
        {
            if (mdp_->goal(*(Ist->second)))
            {
                Ivl->first = 1.0;
                ++Ivl;
                continue;
            }

            double max_prob = -1.0;

            for (GraphMDP<GraphStateWT<GraphStateProbability> >::actions_map_t::const_iterator Iac = Ist->second->get_transitions().begin() ; Iac != Ist->second->get_transitions().end() ; ++Iac)
            {
                double aprob = 0.0;

                for (GraphMDP<GraphStateWT<GraphStateProbability> >::transition_list_t::const_iterator Ieff = Iac->second.begin() ; Ieff != Iac->second.end() ; ++Ieff)
                    aprob += ((*Ieff)->get_probability()) * ((*Ieff)->get_outcome().get_probability());
                
                max_prob = std::max(max_prob, aprob);
            }
            
            Ivl->first = max_prob;
            bellman_error_ = std::max(bellman_error_, std::fabs((Ivl->first) - (Ist->second->get_probability())));
            ++Ivl;
        }
        
        Ivl = record_values_.begin();
        
        for (GraphMDP<GraphStateWT<GraphStateProbability> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
        {
            Ist->second->set_probability(Ivl->first);
            ++Ivl;
        }
        
        if (bellman_error_ < epsilon_)
        {
            probability_converged_ = true;
            bellman_error_ = std::numeric_limits<double>::max();
        }
    }
    else
    {
        for (GraphMDP<GraphStateWT<GraphStateProbability> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
        {
            if ((mdp_->goal(*(Ist->second))) || ((Ivl->first) <= 0.0))
            {
                Ivl->second = 0.0;
                Ist->second->set_policy(*(Ist->second->get_transitions().begin()->first));
                ++Ivl;
                continue;
            }
            
            double max_reward = -std::numeric_limits<double>::max();

            for (GraphMDP<GraphStateWT<GraphStateProbability> >::actions_map_t::const_iterator Iac = Ist->second->get_transitions().begin() ; Iac != Ist->second->get_transitions().end() ; ++Iac)
            {
                double aprob = 0.0;
                double areward = 0.0;

                for (GraphMDP<GraphStateWT<GraphStateProbability> >::transition_list_t::const_iterator Ieff = Iac->second.begin() ; Ieff != Iac->second.end() ; ++Ieff)
                {
                    aprob += ((*Ieff)->get_probability()) * ((*Ieff)->get_outcome().get_probability());
                    areward += (((*Ieff)->get_probability()) * ((*Ieff)->get_outcome().get_probability())) * (((*Ieff)->get_reward()) + ((*Ieff)->get_outcome().get_value()));
                }
                
                areward /= (Ivl->first);
                
                if ((std::fabs(aprob - (Ivl->first)) < epsilon_) && (areward > max_reward))
                {
                    max_reward = areward;
                    Ist->second->set_policy(*(Iac->first));
                }
            }
            
            Ivl->second = max_reward;
            bellman_error_ = std::max(bellman_error_, std::fabs((Ivl->second) - (Ist->second->get_value())));
            ++Ivl;
        }
        
        Ivl = record_values_.begin();
        
        for (GraphMDP<GraphStateWT<GraphStateProbability> >::states_map_t::const_iterator Ist = mdp_->get_states().begin() ; Ist != mdp_->get_states().end() ; ++Ist)
        {
            Ist->second->set_value(Ivl->second);
            ++Ivl;
        }
    }
}


bool GraphGCO::has_converged()
{
	if (probability_converged_ && (bellman_error_ < epsilon_))
		return true;
	else
		return false;
}

