/**
    pso.h
    Particle Swarm Optimization
    
    @author Antoine Passemiers
    @version 1.0 01/08/2019
*/

#include "pso.h"


/**
    Constructs a particle swarm optimizer.

    @param task Task to be performed (either MAXIMIZE or MINIMIZE).
    @param n_particles Number of particles in the swarm.
    @param n_dim Number of dimensions of the search space.
*/
PSO::PSO(short task, size_t n_particles, size_t n_dim) {
    this->initialize(task, n_dim);
}

/**
    Construcs a particle swarm optimizer with specified
    topology.

    @param task Task to be performed (either MAXIMIZE or MINIMIZE).
    @param n_particles Number of particles in the swarm.
    @param n_dim Number of dimensions of the search space.
    @param topology Swarm topology defining the neighbourhood
        of each particle.
*/
PSO::PSO(short task, size_t n_particles, size_t n_dim, short topology) {
    this->n_particles = n_particles;
    this->topology = topology;

    // Assign the proper method pointer
    switch(topology) {
        case TOPOLOGY_ERGODIC:
            this->setNeighborhood = &PSO::createErgodicTopology;
            break;
        case TOPOLOGY_RING:
            this->setNeighborhood = &PSO::createRingTopology;
            break;
        case TOPOLOGY_STAR:
            this->setNeighborhood = &PSO::createStarTopology;
            break;
        default:
            this->setNeighborhood = &PSO::createErgodicTopology;
    }
    this->initialize(task, n_dim);
}

/**
    Initializes the particle swarm optimizer by constructing
    the neighbourhood of each particle, either based on the
    default topology are the requested one.

    @param task Task to be performed (either MAXIMIZE or MINIMIZE).
    @param n_dim Number of dimensions of the search space.
*/
void PSO::initialize(short task, size_t n_dim) {
    this->n_dim = n_dim;
    this->n_eval_without_improvement = 0;

    // Identifier of the next particle to be evaluated:
    // At the moment of the initialization, the particle
    // to be evaluated is the first one.
    this->next_particle_id = 0;

    // Constructs the swarm
    for (size_t i = 0; i < n_particles; i++) {
        swarm.push_back(new Particle(task, n_dim, phi_1, phi_2, inertia));
    }

    // Constructs the neighbourhood of each particle
    (this->*setNeighborhood)();

    // Arbitrarily set the first particle as the currentbest solution.
    // This makes no difference since no particle has
    // been evaluated yet.
    this->global_best = swarm[0]->getPbestSolution();
}

/**
    Creates a ring topology by connecting each particle to the
    particles having an adjacent identifier (modulo the number
    of particles in the swarm).
*/
void PSO::createRingTopology() {
    for (size_t i = 0; i < n_particles; i++) {
        swarm[i]->addNeighbour(swarm[(i - 1) % n_particles]);
        swarm[i]->addNeighbour(swarm[(i + 1) % n_particles]);
    }
}

/**
    Creates a star topology by connecting each particle to the
    particle of identifier 0, which is set arbitrarily as the
    center of the star.
*/
void PSO::createStarTopology() {
    for (size_t i = 1; i < n_particles; i++) {
        swarm[i]->addNeighbour(swarm[0]);
        swarm[0]->addNeighbour(swarm[i]);
    }
}

/**
    Creates an ergodic topology where each particle is connected
    to all other particles in the swarm.
*/
void PSO::createErgodicTopology() {
    for (size_t i = 0; i < n_particles; i++) {
        for (size_t j = 0; j < i; j++) {
            swarm[i]->addNeighbour(swarm[j]);
            swarm[j]->addNeighbour(swarm[i]);
        }
    }
}

/**
    Updates the swarm by setting the best solution
    as the best newly found solution if the latter
    improves the fitness function.
    This method is supposed to be called after e
*/
void PSO::update() {
    bool is_improvement = false;
    this->n_evaluations++;
    for (size_t i = 0; i < n_particles; i++) {
        if (swarm[i]->getPbestEvaluation() > global_best.eval) {
            this->global_best = swarm[i]->getPbestSolution();
            is_improvement = true; // Improved global best solution
        }
    }
    // Update the number of evaluations without improvement
    if (!is_improvement) {
        this->n_eval_without_improvement++;
    } else {
        this->n_eval_without_improvement = 0;
    }
}

/**
    Performs one evaluation of the PSO algorithm.
    If all particles have been evaluated, then positions and
    velocities are being updated.

    @return The next particle to be evaluated.
*/
Particle* PSO::next() {
    // Checks if the PSO algorithm has just started
    if ((this->next_particle_id == 0) && (this->n_evaluations > 0)) {
        // If all particles have been evaluated at least once each,
        // then move them.
        for (size_t i = 0; i < this->n_particles; i++) {
            swarm[i]->move();
        }
        this->n_iterations++;

        // Apply decay to the inertia weight
        this->setInertia(this->inertia * this->decay);
    }
    // Move to the next particle to be evaluated
    Particle* particle = this->swarm[this->next_particle_id];
    this->next_particle_id = (this->next_particle_id + 1) % this->n_particles;
    return particle;
}

/**
    Algorithm termination condition.

    @return Whether the algorithm has converged or should
        be stopped.
*/
bool PSO::terminationCondition() {
    if (this->n_iterations > this->max_iterations) {
        return true;
    } else if (this->n_evaluations > this->max_evaluations) {
        return true;
    } else if (this->n_eval_without_improvement >= this->max_n_eval_without_improvement) {
        return true;
    } else {
        return false;
    }
}

/**
    Setter for the personal influence parameter.

    @param phi_1 New personal influence parameter.
*/
void PSO::setPhi1(double phi_1) {
    this->phi_1 = phi_1; // Update PSO
    for (size_t i = 0; i < n_particles; i++) {
        swarm[i]->phi_1 = phi_1; // Update particle
    }
}

/**
    Setter for the social influence parameter.

    @param phi_2 New social influence parameter.
*/
void PSO::setPhi2(double phi_2) {
    this->phi_2 = phi_2; // Update PSO
    for (size_t i = 0; i < n_particles; i++) {
        swarm[i]->phi_2 = phi_2; // Update particle
    }
}

/**
    Setter for the inertia weight parameter.

    @param inertia New inertia weight parameter.
*/
void PSO::setInertia(double inertia) {
    this->inertia = inertia; // Update PSO
    for (size_t i = 0; i < n_particles; i++) {
        swarm[i]->inertia = inertia; // Update particle
    }
}
