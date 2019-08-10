/**
    speed.cpp
    Desired speed module
    
    @author Antoine Passemiers
    @version 1.0 05/08/2019
*/

#include "speed.h"


/**
    Constructs the multi-layer perceptron with 3
    layers and 7 input neurons per hidden layer.
*/
TargetSpeedModule::TargetSpeedModule() {
    this->mlp = new MLP(7);
    this->mlp->addFullyConnectedLayer(7, 7);
    this->mlp->addActivation(ACTIVATION_TANH);
    this->mlp->addFullyConnectedLayer(7, 7);
    this->mlp->addActivation(ACTIVATION_TANH);
    this->mlp->addFullyConnectedLayer(7, 1);
    // Ensures that the output is in the range [0, 1]
    this->mlp->addActivation(ACTIVATION_CLIPPING);
}

/**
    Outputs the desired speed based on sensory data.

    @param cs Current car state.
    @return Desired speed.
*/
double TargetSpeedModule::control(CarState &cs) {
    // Normalize sensor data and pass them to the network
    for (int i = -3; i < 4; i++) {
        this->mlp->in(i + 3) = cs.track[FRONT + i] / 200.0;
    }

    // Forward pass
    this->mlp->forward();

    // Retrieve the output value and map it to actual speed
    double output = this->mlp->out(0);
    double speed = output * (this->max_speed - this->min_speed) + this->min_speed;
    if (cs.track[FRONT] >= 100) speed = 300.0;
    return speed;
}

/**
    Number of module parameters. Parameters include
    MLP weights and the two bounds on speed values.

    @return Number of module parameters.
*/
size_t TargetSpeedModule::getNumberOfParameters() {
    int n = this->mlp->getNumberOfParameters();
    return n + 2;
}

/**
    @return Lower bounds on the module parameters.
*/
Eigen::VectorXd TargetSpeedModule::getLowerBounds() {
    int n = this->mlp->getNumberOfParameters();
    Eigen::VectorXd lbs = Eigen::VectorXd::Zero(n + 2);
    for (int i = 0; i < n; i++) {
        // lb chosen such that the corresponding uniform
        // distribution has the same standard deviation
        // as a gaussian distribution used for Xavier
        // initialization with n_in + n_out = 14.
        // sigma^2 = (1/14)^2 = (1/12) * (b - a)^2,
        // where b = -a.
        lbs[i] = -std::sqrt(6.0 / (14.0 * 14.0));
    }
    lbs[n] = 0.0;
    lbs[n + 1] = 100.0;
    return lbs;
}

/**
    @return Upper bounds on the module parameters.
*/
Eigen::VectorXd TargetSpeedModule::getUpperBounds() {
    int n = this->mlp->getNumberOfParameters();
    Eigen::VectorXd ubs = Eigen::VectorXd::Zero(n + 2);
    for (int i = 0; i < n; i++) {
        // lb chosen such that the corresponding uniform
        // distribution has the same standard deviation
        // as a gaussian distribution used for Xavier
        // initialization with n_in + n_out = 14.
        // sigma^2 = (1/14)^2 = (1/12) * (b - a)^2,
        // where b = -a.
        ubs[i] = std::sqrt(6.0 / (14.0 * 14.0));
    }
    ubs[n] = 70.0;
    ubs[n + 1] = 350.0;
    return ubs;
}

/**
    Gets current values of module parameters.

    @return Current values of module parameters.
*/
Eigen::VectorXd TargetSpeedModule::getParameters() {
    int n = this->mlp->getNumberOfParameters();
    Eigen::VectorXd parameters = Eigen::VectorXd::Zero(n + 2);
    Eigen::VectorXd weights = this->mlp->getWeights();
    for (int i = 0; i < n; i++) {
        parameters[i] = weights[i];
    }
    parameters[n] = this->min_speed;
    parameters[n + 1] = this->max_speed;
    return parameters;
}

/**
    Sets current values of module parameters.

    @param Current values of module parameters.
*/
void TargetSpeedModule::setParameters(Eigen::VectorXd parameters) {
    int n = this->mlp->getNumberOfParameters();
    Eigen::VectorXd weights = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; i++) weights[i] = parameters[i];
    this->mlp->setWeights(weights);
    this->min_speed = parameters[n];
    this->max_speed = parameters[n + 1];
}
