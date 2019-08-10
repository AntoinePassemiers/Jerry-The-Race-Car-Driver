/**
    particle.cpp
    Component for particle swarms
    
    @author Antoine Passemiers
    @version 1.0 01/08/2019
*/

#include "particle.h"
#include "pso.h"


/**
    Constructs a particle with specified parameters.

    @param pso The swarm to which belongs the particle.
    @param task The task to be performed,
        either MAXIMIZE or MINIMIZE.
    @param n_dim The number of dimensions of the search space.
    @param phi_1 Personal influence parameter.
    @param phi_2 Social influence parameter.
    @param inertia Inertia weight.
*/
Particle::Particle(short task, size_t n_dim,
                   double phi_1, double phi_2, double inertia) {
    this->task = task;
    this->n_dim = n_dim;
    this->current.x = Eigen::VectorXd::Zero(this->n_dim);
    this->pbest.x = Eigen::VectorXd::Zero(this->n_dim);
    this->gbest.x = Eigen::VectorXd::Zero(this->n_dim);
    this->current.eval = (task == MAXIMIZE) ? -DBL_MAX : DBL_MAX;
    this->pbest.eval = (task == MAXIMIZE) ? -DBL_MAX : DBL_MAX;
    this->gbest.eval = (task == MAXIMIZE) ? -DBL_MAX : DBL_MAX;

    this->velocity = Eigen::VectorXd::Zero(this->n_dim);
    this->phi_1 = phi_1;
    this->phi_2 = phi_2;
    this->inertia = inertia;
}

/**
    Copy-constructs a particle with default parameters.

    @param other The particle to be copied.
*/
Particle::Particle(const Particle &other) {
    *this = other; // implicit call to the copy assignment operator
}

/**
    Copy assignment operator.

    @param other The partice to be copied.
    @return The current particle.
*/
Particle& Particle::operator=(const Particle &other) {
    if (this != &other) {

        // Copy all attributes
        this->task = other.task;
        this->n_dim = other.n_dim;
        this->current = other.current;
        this->pbest = other.pbest;
        this->gbest = other.gbest;
        this->velocity = other.velocity;
        this->inertia = other.inertia;
        this->phi_1 = other.phi_1;
        this->phi_2 = other.phi_2;
    }
    return *this;
}

// TODO
void Particle::initialize(Eigen::VectorXd &lbs, Eigen::VectorXd &ubs) {
    this->lbs = lbs;
    this->ubs = ubs;

    Eigen::VectorXd position = randUniform(this->n_dim).cwiseProduct((ubs - lbs) + lbs);
    this->current.x = position;
    this->current.eval = TO_BE_EVALUATED; // No evaluation yet

    // Speed is initialized randomly on a scale defined by the
    // difference between the lower bound and the upper bound.
    Eigen::VectorXd range = ubs - lbs;
    this->velocity = Eigen::VectorXd::Random(position.size()).cwiseProduct(range);
}

/**
    Updates speed and position of the particle, based
    on the new local best and personal best solutions.
*/
void Particle::move() {
    Eigen::VectorXd pb = this->pbest.x;
    Eigen::VectorXd lb = this->gbest.x;
    Eigen::VectorXd x = this->current.x;

    // Apply decay/inertia to velocity
    this->velocity *= this->inertia;

    // Add personal influence to velocity
    Eigen::VectorXd u1 = randUniform(this->n_dim);
    this->velocity += this->phi_1 * u1.cwiseProduct(pb - x);

    // Add social influence to velocity
    Eigen::VectorXd u2 = randUniform(this->n_dim);
    this->velocity += this->phi_2 * u2.cwiseProduct(lb - x);

    // Update position
    this->current.x += this->velocity;
    this->current.eval = TO_BE_EVALUATED;

    // Make sure that the new position stays in the bounds
    this->current.x = this->current.x.cwiseMax(this->lbs);
    this->current.x = this->current.x.cwiseMin(this->ubs);
}

/**
    Looks in the neighbourhood for a new local best solution.
    Current particle does not belong to its own neighbourhood
    and does local best solution cannot be the same as personal
    best solution.
*/
void Particle::checkNeibourhood() {
    int i = 0;
    int best_index = 0;
    double best_score = (this->task == MAXIMIZE) ? -DBL_MAX : DBL_MAX;
    for (auto it = neighbours.begin(); it != neighbours.end(); it++) {
        Particle* neighbour = *it;
        double score = neighbour->getPbestEvaluation();
        if ((this->task == MAXIMIZE) != (score < best_score)) {
            best_score = score;
            best_index = i;
        }
        i++;
    }
    this->gbest = neighbours[best_index]->getPbestSolution();
}

/**
    Assigns an actual value to the objective function
    for the current position of the particle. Because the
    This method is useful when the main loop of the program
    is dedicated to something else than the particle swarm optimizer.
    In the present case, the TORCS-SCR client is responsible for
    driving the car and pass the evaluation via this method once
    it is available (e.g. at the end of a race).

    @param eval Evaluation of the fitness function.
*/
void Particle::setEvaluation(double eval) {
    this->current.eval = eval;
    if (eval > this->pbest.eval) {
        this->pbest = this->current;
    }

    // Updates second-order neighbourhood
    this->checkNeibourhood();
    for (auto it = neighbours.begin(); it != neighbours.end(); it++) {
        Particle* neighbour = *it;
        neighbour->checkNeibourhood();
    }
}

/**
    Dynamically adds neighbour to the neighbourhood of
    the current particle.

    @param other New neighbour.
*/
void Particle::addNeighbour(Particle* neighbour) {
    this->neighbours.push_back(neighbour);
}
