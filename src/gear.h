/**
    gear.h
    Gear-controlling module
    
    @author Antoine Passemiers
    @version 1.0 05/08/2019
*/

#ifndef GEAR_H__
#define GEAR_H__

#include <cstdlib>

#include "carcontrol.h"
#include "carstate.h"
#include "module.h"


class GearModule : Module {
private:

    // Gear increase and decrease thresholds
    int GI[6] = { 8000, 8000, 8000, 8000, 8000,    0 };
    int GD[6] = {    0, 2500, 3000, 3000, 3500, 3500 };

    // Counter for detecting whether the car is stuck.
    // The counter is incremented at each step where the
    // car points in a wrong direction.
    // The car is considered to be stuck when the counter
    // exceeds a given threshold.
    size_t stuck;

    // Whether the car is currently trying to get unstuck
    bool getting_unstuck;

public:
    // Constructor and destructor
    GearModule();
    ~GearModule() = default;

    // Checks whether the car is stuck
    bool checkIfStuck(CarState &cs);

    // Selects the gear
    int control(CarState &cs);

    // Abstract methods
    virtual size_t getNumberOfParameters();
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual Eigen::VectorXd getParameters();
    virtual void setParameters(Eigen::VectorXd parameters);
};


#endif // GEAR_H__
