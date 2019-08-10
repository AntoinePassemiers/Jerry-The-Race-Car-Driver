/**
    steering.h
    Steering control module
    
    @author Antoine Passemiers
    @version 1.0 05/08/2019
*/

#ifndef STEERING_H__
#define STEERING_H__

#include <cstdlib>

#include "carcontrol.h"
#include "carstate.h"
#include "module.h"


class SteeringControlModule : Module {
private:
    // Steer lock
    static constexpr double STEER_LOCK = 0.785398;

    // Index of the front sensor
    static constexpr int FRONT = 9;

    // Sensor weights
    Eigen::VectorXd weights;

public:
    // Constructor and destructor
    SteeringControlModule();
    ~SteeringControlModule() = default;

    // Outputs the steering value based on current car state
    double control(CarState &cs);

    // Checks whether the car is on track
    bool isOnTrack(CarState &cs);

    // Abstract methods
    virtual size_t getNumberOfParameters();
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual Eigen::VectorXd getParameters();
    virtual void setParameters(Eigen::VectorXd parameters);
};


#endif // STEERING_H__
