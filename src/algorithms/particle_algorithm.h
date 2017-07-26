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

#ifndef PARTICLE_ALGORITHM_H_
#define PARTICLE_ALGORITHM_H_

#include "base_algorithm.h"

template <class Tstate>
class ParticleAlgorithm : public virtual BaseAlgorithm
{
public :
	class Particle
	{
	private :
		Tstate current_state_;
		
	public :
		Particle();
		~Particle() {}
		
		void set_state(Tstate& st);
		inline const Tstate& get_state() const {return current_state_;}
		template <class Tmdp> void change_state(Tmdp& mdp);
	};
	
	typedef std::list<Particle> particles_list_t;
	
protected :
	particles_list_t particles_;
	double probabilistic_threshold_;
	unsigned int trajectories_max_depth_;
	
	/* Constructs an algorithm */
	ParticleAlgorithm(const Problem& pb, double epsilon, double discount_factor, double probabilistic_threshold = 0.1, unsigned int nb_of_particles = 10, unsigned int trajectories_max_depth = 100); // throws
	
	/* Destructs this algorithm */
	virtual ~ParticleAlgorithm() {}
	
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

#endif /*PARTICLE_ALGORITHM_H_*/
