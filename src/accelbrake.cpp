/**
    accelbrake.cpp
    Acceleration-brake module
    
    @author Antoine Passemiers
    @version 1.0 05/08/2019
*/

#include "accelbrake.h"


/**
    Process car state and outputs a control value for the
    brake/acceleration. The value ranges from 0 to 1,
    where 0 corresponds to full brake and 1 corresponds to
    full acceleration. If the value is 0.5, then no action is
    performed.

    @param cs The current car state.
    @param target_speed The desired speed, as outputed by the
        target speed module.
    @return The acceleration/brake control value.
*/
double AccelBrakeModule::control(CarState &cs, double target_speed) {
    double accelbrake;
    if (cs.gear == -1) {
        accelbrake = 1.0;
    } else {
        // Speed control value: close to 2 when the deviation from
        // the desired speed is very large and 0 in the opposite case
        accelbrake = 2.0 / (1.0 + std::exp(cs.getSpeed() - target_speed));

        // ABS filtering for preventing the car from slipping
        if (cs.getSpeed() - cs.getWheelsSpeed() > this->threshold) {
            accelbrake -= (cs.getSpeed() - cs.getWheelsSpeed() - this->threshold) / 5.0;
        }
        accelbrake /= 2.0; // Normalize the output value
    }
    return accelbrake;
}

/**
    Number of parameters in the module.
    The only parameter is the ABS filtering threshold.

    @return The number of parameters.
*/
size_t AccelBrakeModule::getNumberOfParameters() {
    return 1;
}

/**
    @return Lower bounds on the module parameters.
*/
Eigen::VectorXd AccelBrakeModule::getLowerBounds() {
    Eigen::VectorXd lbs = Eigen::VectorXd::Zero(1);
    lbs[0] = this->threshold_lb;
    return lbs;
}

/**
    @return Upper bounds on the module parameters.
*/
Eigen::VectorXd AccelBrakeModule::getUpperBounds() {
    Eigen::VectorXd ubs = Eigen::VectorXd::Zero(1);
    ubs[0] = this->threshold_ub;
    return ubs;
}

/**
    Gets current values of module parameters.

    @return Current values of module parameters.
*/
Eigen::VectorXd AccelBrakeModule::getParameters() {
    Eigen::VectorXd parameters = Eigen::VectorXd::Zero(1);
    parameters[0] = this->threshold;
    return parameters;
}

/**
    Sets current values of module parameters.

    @param Current values of module parameters.
*/
void AccelBrakeModule::setParameters(Eigen::VectorXd parameters) {
    this->threshold = parameters[0];
}
