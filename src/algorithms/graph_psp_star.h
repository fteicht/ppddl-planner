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

#ifndef GRAPH_PSP_STAR_H_
#define GRAPH_PSP_STAR_H_

#include <set>
#include "heuristic_algorithm.h"

class Particle
{
private :
	GraphStateWT<> * current_state_;
	
public :
	Particle() : current_state_(NULL) {}
	~Particle() {}
	
	void set_state(GraphStateWT<>& st) {current_state_ = &st;}
	inline GraphStateWT<>* get_state() const {return current_state_;}
	void change_state(GraphMDP<GraphStateWT<> >& mdp);
};

class GraphPSPstar : public GraphHeuristicAlgorithm<GraphStateWT<> >
{
public :
	typedef std::list<Particle> particles_list_t;
	typedef std::set<GraphStateWT<>*> states_set_t;
	typedef std::map<GraphStateWT<>*, double> particles_cnt_t;
	
private :
	double probabilistic_threshold_;
	particles_list_t particles_;
	unsigned int trajectories_max_depth_;
	states_set_t reachable_states_;
	bool continue_search_;
	
	void compute_reachability();
	double compute_exit_probability();
	
public :
	/* Constructs an algorithm */
	GraphPSPstar(const Problem& pb, double epsilon, double discount_factor, heuristic_t heuristic_type,
					determinization_t determinization_type, deterministic_planner_t deterministic_planner_type,
					double probabilistic_threshold = 0.1, unsigned int nb_of_particles = 10, unsigned int trajectories_max_depth = 100); // throws
	
	/* Destructs this algorithm */
	virtual ~GraphPSPstar() {}
	
	/* Initialize the optimization process with a given intial state */
	virtual void solve_initialize(const PddlState& st);
	
	/* Progress in the optimization: perform one step of the main algorithm's loop */
	virtual void solve_progress();
	
	/* Return TRUE if the algorithm has converged */
	virtual bool has_converged();
	
	/* Sets the probabilistic threshold */
	void set_probabilistic_threshold(double probabilistic_threshold); // throws
	
	/* Gets the probabilistic threshold */
	inline double get_probabilistic_threshold() const {return probabilistic_threshold_;}
	
	/* Sets the number of particles */
	void set_nb_of_particles(unsigned int nb_of_particles) {particles_.resize(nb_of_particles);}
	
	/* Gets the number of particles */
	unsigned int get_nb_of_particles() const {return particles_.size();}
	
	/* Set trajectories' max depth */
	inline void set_trajectories_max_depth(unsigned int trajectories_max_depth) {trajectories_max_depth_ = trajectories_max_depth;}
	
	/* Gets trajectories' max depth */
	inline unsigned int get_trajectories_max_depth() const {return trajectories_max_depth_;}
};

#endif /*GRAPH_PSP_STAR_H_*/
