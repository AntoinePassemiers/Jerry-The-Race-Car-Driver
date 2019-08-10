/**
    particle.h
    Component for particle swarms
    
    @author Antoine Passemiers
    @version 1.0 01/08/2019
*/

#ifndef PARTICLE_H__
#define PARTICLE_H__

#include <Eigen/Core>
#include <cmath>
#include <float.h>
#include <vector>

#include "utils.h"

// Dummy value for the objective function
#define TO_BE_EVALUATED -1

// Type of optimization task
#define MAXIMIZE 0
#define MINIMIZE 1


struct Solution {

	// Values of the solution components
	Eigen::VectorXd x;

	// Value of the objective function
	double eval;

	// Copy assignment operator
	Solution& operator=(const Solution& sol) {
		if (this != &sol) {
			this->x = sol.x;
			this->eval = sol.eval;
		}
		return *this;
	};
};


// Forward declaration of PSO
class PSO;


class Particle {

public:

    // Task to be performed
    short task = MAXIMIZE;

	// Number of controller parameters
	long int n_dim;

	// Current velocity
	Eigen::VectorXd velocity;

	// Current position
	struct Solution current;

	// Personal best position so far
	struct Solution pbest;

	// Best position of the whole neighborhood so far
	struct Solution gbest;

	// Neighborhood
	std::vector<Particle*> neighbours;

	// Lower and upper bounds of particle positions.
    Eigen::VectorXd lbs;
    Eigen::VectorXd ubs;

    // Inertia term
    double inertia;

    // Personal and social influence parameters
    double phi_1;
    double phi_2;

    // Constructors, copy assignment operator and destructor
	Particle (short task, size_t n_dim,
              double phi_1, double phi_2, double inertia);
	Particle (const Particle &p);
	Particle& operator=(const Particle& p);
	~Particle() = default;

    // Initializes position and speed
    void initialize(Eigen::VectorXd &lbs, Eigen::VectorXd &ubs);

    // Sets the value of the objective function at current position
	void setEvaluation(double eval);

    // Updates speed and position
	void move();

    // Neighbourhood-related methods
	void addNeighbour(Particle* neighbour);
	void checkNeibourhood();
	void updateGlobalBest(Eigen::VectorXd &x, double eval);

	// Getters
	Eigen::VectorXd& getCurrentPosition() { return this->current.x; }
	double getCurrentEvaluation() { return this->current.eval; }
	Solution& getCurrentSolution() { return this->current; }
	Eigen::VectorXd& getPbestPosition() { return this->pbest.x; }
	double getPbestEvaluation() { return this->pbest.eval; }
	Solution& getPbestSolution() { return this->pbest; }
};

#endif // PARTICLE_H__
