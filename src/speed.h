/**
    speed.h
    Desired speed module
    
    @author Antoine Passemiers
    @version 1.0 05/08/2019
*/

#ifndef SPEED_H__
#define SPEED_H__

#include "carstate.h"
#include "mlp.h"
#include "module.h"


class TargetSpeedModule : Module {
private:
    // Index of the front sensor
    static constexpr int FRONT = 9;

    // Multi-layer perceptron
    MLP* mlp;

    // Minimum and maximum desired speed
    double min_speed;
    double max_speed;

public:
    // Constructor and destructor
    TargetSpeedModule();
    ~TargetSpeedModule() {};

    // Outputs the desired speed
    double control(CarState &cs);

    // Abstract method
    virtual size_t getNumberOfParameters();
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual Eigen::VectorXd getParameters();
    virtual void setParameters(Eigen::VectorXd parameters);
};


#endif // SPEED_H__
