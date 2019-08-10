/**
    gear.cpp
    Gear-controlling module
    
    @author Antoine Passemiers
    @version 1.0 05/08/2019
*/

#include "gear.h"


/**
    Constructs the gear module.
*/
GearModule::GearModule() {
    this->stuck = 0; // the car is not stuck yet
    this->getting_unstuck = false;
}

/**
    Checks whether the car is currently stuck.

    @param cs Current car state.
    @return Whether the car is stuck
*/
bool GearModule::checkIfStuck(CarState &cs) {
    // Checks whether the car is stuck
    if (std::abs(cs.angle) > M_PI / 6.0) { // Not aligned with the road
        this->stuck++;
    } else {
        this->stuck = 0;
    }
    if (this->stuck >= 25) { // If the angle was off for too long
        this->getting_unstuck = true;
    }

    // Checks whether the car should try to get unstuck
    if (this->getting_unstuck) {
        double front = cs.track[9];
        // Don't try to get unstuck if there is an obstacle
        if ((cs.angle * cs.trackPos > 0) || ((front > 10) && (std::abs(M_PI) < 2.0))) {
            this->getting_unstuck = false;
        }
    }
    return this->getting_unstuck;
}

/**
    Selects the gear based on current car state.
    Except for reverse gear, gear changing is done
    if either a lower or an upper threshold is reached.

    @param cs Current car state.
    @return Gear selection
*/
int GearModule::control(CarState &cs) {
    // Get gear changing thresholds for current gear selection
    int gear;
    int gd = GearModule::GI[cs.gear + 1];
    int gi = GearModule::GD[cs.gear + 1];

    if (this->checkIfStuck(cs)) {
        gear = -1; // Reverse gear
    } else if ((cs.rpm > gi) && (cs.gear < 6)) {
        gear = cs.gear + 1; // Increase gear
    } else if ((cs.rpm < gd) && (cs.gear > 1)) {
        gear = cs.gear - 1; // Decrease gear
    } else {
        gear = cs.gear; // Keep unchanged
    }
    return gear;
}

/**
    Number of module parameters. There are 6
    gear increase thresholds and 6 gear decrease thresholds.

    @return Number of module parameters.
*/
size_t GearModule::getNumberOfParameters() {
    return 12;
}

/**
    @return Lower bounds on the module parameters.
*/
Eigen::VectorXd GearModule::getLowerBounds() {
    Eigen::VectorXd lbs = Eigen::VectorXd::Zero(12);
    for (int i = 0; i < 6; i++) lbs[i] = 3000;
    for (int i = 6; i < 12; i++) lbs[i] = 1000;
    return lbs;
}

/**
    @return Upper bounds on the module parameters.
*/
Eigen::VectorXd GearModule::getUpperBounds() {
    Eigen::VectorXd ubs = Eigen::VectorXd::Zero(12);
    for (int i = 0; i < 6; i++) ubs[i] = 8000;
    for (int i = 6; i < 12; i++) ubs[i] = 4000;
    return ubs;
}

/**
    Gets current values of module parameters.

    @return Current values of module parameters.
*/
Eigen::VectorXd GearModule::getParameters() {
    Eigen::VectorXd parameters = Eigen::VectorXd::Zero(12);
    for (int i = 0; i < 6; i++) parameters[i] = GI[i];
    for (int i = 6; i < 12; i++) parameters[i] = GD[i - 6];
    return parameters;
}

/**
    Sets current values of module parameters.

    @param Current values of module parameters.
*/
void GearModule::setParameters(Eigen::VectorXd parameters) {
    for (int i = 0; i < 6; i++) parameters[i] = GI[i];
    for (int i = 6; i < 12; i++) parameters[i] = GD[i - 6];
}
