/**
    pso.h
    Particle Swarm Optimization
    
    @author Antoine Passemiers
    @version 1.0 01/08/2019
*/

#ifndef PSO_H__
#define PSO_H__

#include <Eigen/Core>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <limits.h>
#include <string.h>
#include <vector>
#include <float.h>

#include "particle.h"


// Swarm topologies
#define TOPOLOGY_ERGODIC 0
#define TOPOLOGY_RING    1
#define TOPOLOGY_STAR    2


class PSO {

public:

    // The task to be performed:
    // Either MAXIMIZE or MINIMIZE.
    short task = MAXIMIZE;
    
    // Neighbourhood topology and pointer to neighbourhood
    // creation method. Topology can be either 'TOPOLOGY_ERGODIC',
    // 'TOPOLOGY_RING' or 'TOPOLOGY_STAR'.
    // Default topology is 'TOPOLOGY_ERGODIC'
    short topology = TOPOLOGY_ERGODIC;
    void (PSO::*setNeighborhood)() = &PSO::createErgodicTopology;

    // Number of iterations and evaluations of the objective
    // function done so far
    size_t n_iterations = 0;
    size_t n_evaluations = 0;

    // Number of evaluations without improvement
    // of the global best solution
    size_t n_eval_without_improvement = 0;
    size_t max_n_eval_without_improvement = 300;

    // Maximum number of iterations and evaluations of the
    // objective function allowed
    size_t max_iterations = 1000;
    size_t max_evaluations = 10000;

    // Number of dimensions
    size_t n_dim;

    // Number of particles
    size_t n_particles = 25;

    // Inertia weight
    double inertia = 1.0;

    // Inertia decay
    double decay = 0.98;

    // Personal and social influence parameters
    double phi_1 = 1.0;
    double phi_2 = 1.0;

    // Swarm
    std::vector<Particle*> swarm;

    // Identifier of the next particle to be evaluated.
    // Each time this identifier becomes 0, it means
    // that all particles have been evaluated and can
    // thus be moved all at once.
    int next_particle_id = 0;

    // Current best solution found so far
    struct Solution global_best;

    // Constructors and destructor
    PSO(short task, size_t n_particles, size_t n_dim);
    PSO(short task, size_t n_particles, size_t n_dim, short topology);
    ~PSO() = default;

    // Topology creation methods
    void createRingTopology();
    void createStarTopology();
    void createErgodicTopology();

    // PSO initialization
    void initialize(short task, size_t n_dim);

    // Get the next particle which position has
    // to be evaluated
    Particle* next();
    void update();

    // Convergence condition
    bool terminationCondition();

    // Getters
    Eigen::VectorXd& getBestPosition() { return this->global_best.x; }

    // Setters
    void setPhi1(double phi_1);
    void setPhi2(double phi_2);
    void setInertia(double inertia);
};


#endif // PSO_H__
