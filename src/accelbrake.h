/**
    accelbrake.h
    Acceleration-brake module
    
    @author Antoine Passemiers
    @version 1.0 05/08/2019
*/

#ifndef ACCELBRAKE_H__
#define ACCELBRAKE_H__

#include <Eigen/Core>

#include "carstate.h"
#include "module.h"


class AccelBrakeModule : Module {
private:

    // Lower bound on the ABS filtering threshold
    static constexpr double threshold_lb = 1.0;

    // Upper bound on the ABS filtering threshold
    static constexpr double threshold_ub = 2.0;

    // ABS filtering threshold
    double threshold;
public:

    // Constructor
    AccelBrakeModule() = default;

    // Outputs an acceleration/brake control parameter
    // based on sensory data
    double control(CarState &cs, double target_speed);

    // abstract methods
    virtual size_t getNumberOfParameters();
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual Eigen::VectorXd getParameters();
    virtual void setParameters(Eigen::VectorXd parameters);
};


#endif // ACCELBRAKE_H__
