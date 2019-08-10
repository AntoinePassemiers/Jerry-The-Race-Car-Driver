/**
    opponents.h
    Opponents modifier module
    
    @author Antoine Passemiers
    @version 1.0 05/08/2019
*/

#ifndef OPPONENTS_H__
#define OPPONENTS_H__

#include <Eigen/Core>

#include "carstate.h"
#include "module.h"


class OpponentsModule : Module {
private:
    // Index of the front sensor
    static constexpr int FRONT = 18;

    // Tolerance thresholds of sensors for braking
    //                     +-40° +-30° +-20° +-10°  0°  
    double tol_brake[5] = { 6.0,  6.5,  7.0,  7.5, 8.0 };

    // Tolerance thresholds and increments for overtaking
    //                          > 50°  +-50°  +-40°  +-30°  +-20°  < 20°
    double tol_overtake[6] = {   10.,   12.,   14.,   16.,   18.,    20. };
    double inc_overtake[6] = {  0.10,  0.12,  0.14,  0.16,  0.18,  0.20 };
public:
    // Constructor
    OpponentsModule() = default;

    // Updates car control based on opponents sensors
    void control(CarState &cs, double &steer, double &accelbrake);

    // Checks whether an opponent is close to the car
    bool violatedSecurityDistance(CarState &cs);

    // Abstract methods
    virtual size_t getNumberOfParameters();
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual Eigen::VectorXd getParameters();
    virtual void setParameters(Eigen::VectorXd parameters);
};


#endif // OPPONENTS_H__
