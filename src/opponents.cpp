/**
    opponents.cpp
    Opponents modifier module
    
    @author Antoine Passemiers
    @version 1.0 05/08/2019
*/

#include "opponents.h"


/**
    Checks whether an opponent is close to the car.

    @param cs Current car state.
    @return Whether the security distance is violated.
*/
bool OpponentsModule::violatedSecurityDistance(CarState &cs) {
    bool violated = false;
    for (int i = -4; i < 5; i++) {
        // Check tolerance threshold for each sensor
        if (cs.opponents[FRONT + i] < this->tol_brake[i + 4]) {
            violated = true;
            break;
        }
    }
    return violated;
}

/**
    Updates car control based on opponents sensors.

    @param cs Current car state.
    @param steer Steering value (to be updated).
    @param accelbrale Accel/brake control value (to be updated).
*/
void OpponentsModule::control(CarState &cs, double &steer, double &accelbrake) {
    // Decelerate if security distance is being violated
    if ((cs.getSpeed() > 70) && (this->violatedSecurityDistance(cs))) {
        accelbrake = std::max(0.0, accelbrake - 0.5);
    }

    // Apply increments to the current steering value based on
    // opponents sensors
    for (int i = -10; i < 11; i++) {
        double sign = (i < 0) ? -1.0 : 1.0;
        if (std::abs(i) > 5) { // Special case: sensors ranging from 60° to 100°
            if (cs.opponents[FRONT + i] < this->tol_overtake[0]) {
                steer += -sign * this->inc_overtake[0];
            }
        } else{ // Check sensors ranging from 0° to 50°
            if (cs.opponents[FRONT + i] < this->tol_overtake[i + 5]) {
                steer += -sign * this->inc_overtake[i + 5];
            }
        }
    }
}

/**
    Number of module parameters. There are 5 brake tolerance
    thresholds, 6 overtake tolerance thresholds and 6 values
    for incrementing the steering.

    @return Number of module parameters.
*/
size_t OpponentsModule::getNumberOfParameters() {
    return 17;
}

/**
    @return Lower bounds on the module parameters.
*/
Eigen::VectorXd OpponentsModule::getLowerBounds() {
    return Eigen::VectorXd::Zero(17);
}

/**
    @return Upper bounds on the module parameters.
*/
Eigen::VectorXd OpponentsModule::getUpperBounds() {
    Eigen::VectorXd ubs = Eigen::VectorXd::Zero(17);
    for (int i = 0; i < 11; i++) ubs[i] = 20.0;
    for (int i = 11; i < 17; i++) ubs[i] = 0.30;
    return ubs;
}

/**
    Gets current values of module parameters.

    @return Current values of module parameters.
*/
Eigen::VectorXd OpponentsModule::getParameters() {
    Eigen::VectorXd parameters = Eigen::VectorXd::Zero(12);
    for (int i = 0; i < 5; i++) parameters[i] = this->tol_brake[i];
    for (int i = 5; i < 11; i++) parameters[i] = this->tol_overtake[i - 5];
    for (int i = 11; i < 17; i++) parameters[i] = this->inc_overtake[i - 11];
    return parameters;
}

/**
    Sets current values of module parameters.

    @param Current values of module parameters.
*/
void OpponentsModule::setParameters(Eigen::VectorXd parameters) {
    for (int i = 0; i < 5; i++) this->tol_brake[i] = parameters[i];
    for (int i = 5; i < 11; i++) this->tol_overtake[i - 5] = parameters[i];
    for (int i = 11; i < 17; i++) this->inc_overtake[i - 11] = parameters[i];
}
