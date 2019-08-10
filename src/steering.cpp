/**
    steering.cpp
    Steering control module
    
    @author Antoine Passemiers
    @version 1.0 05/08/2019
*/

#include "steering.h"


/**
    Constructs the steering control module.
*/
SteeringControlModule::SteeringControlModule() {
    // Allocate space for parameters
    this->weights = Eigen::VectorXd::Zero(9);
}

/**
    Outputs the steering value based on current
    car state.

    @param cs Current car state.
    @return Steering value.
*/
double SteeringControlModule::control(CarState &cs) {
    double steer;
    if (cs.gear == -1) {
        // Reversed movement
        steer = -cs.angle / STEER_LOCK;
    } else if (this->isOnTrack(cs)) { // car is no track
        double front = cs.track[FRONT];

        // Reduce steering if straight line
        double f0 = (front >= 100.0) ? 0.2 : 1.0;

        // Compute steering value
        double norm = 0.0;
        steer = 0.0;
        for (int i = -4; i < 5; i++) {
            steer += (cs.track[FRONT + i] * this->weights[i + 4]);
            norm += cs.track[FRONT + i];
        }
        steer *= (f0 / norm);
    } else {
        // Car is out of track
        steer = (cs.angle - cs.trackPos * 0.5) / STEER_LOCK;
    }
    return steer;
}

/**
    Checs whether the car is on track.

    @param cs Current car state.
    @return Whether the car is on track.
*/
bool SteeringControlModule::isOnTrack(CarState &cs) {
    return (std::abs(cs.trackPos) > 1);
}

/**
    Number of module parameters (sensor weights).

    @param Number of module parameters.
*/
size_t SteeringControlModule::getNumberOfParameters() {
    return 9;
}

/**
    @return Lower bounds on the module parameters.
*/
Eigen::VectorXd SteeringControlModule::getLowerBounds() {
    Eigen::VectorXd lbs = Eigen::VectorXd::Zero(9);
    for (int i = -4; i < 5; i++) {
        lbs[i + 4] = (i * 0.5) - 0.5;
    }
    return lbs;
}

/**
    @return Upper bounds on the module parameters.
*/
Eigen::VectorXd SteeringControlModule::getUpperBounds() {
    Eigen::VectorXd ubs = Eigen::VectorXd::Zero(9);
    for (int i = -4; i < 5; i++) {
        ubs[i + 4] = (i * 0.5) + 0.5;
    }
    return ubs;
}

/**
    Gets current values of module parameters.

    @return Current values of module parameters.
*/
Eigen::VectorXd SteeringControlModule::getParameters() {
    return this->weights;
}

/**
    Sets current values of module parameters.

    @param Current values of module parameters.
*/
void SteeringControlModule::setParameters(Eigen::VectorXd parameters) {
    this->weights = parameters;
}
