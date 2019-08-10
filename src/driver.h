/**
    driver.h
    Interface between PSO algorithm and TORCS
    
    @author Antoine Passemiers
    @version 1.0 01/08/2019
*/

#ifndef DRIVER_H__
#define DRIVER_H__

#include <Eigen/Core>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "accelbrake.h"
#include "carcontrol.h"
#include "carstate.h"
#include "gear.h"
#include "mlp.h"
#include "opponents.h"
#include "particle.h"
#include "pso.h"
#include "speed.h"
#include "steering.h"


class Controller {
private:

    // Whether to train the driver using a PSO
    bool is_training = true;

    // Path to the folder where to save parameters
    std::string model_path = ".";

    // Particle swarm optimizer
    PSO* pso;

    // Next particle which position is to be evaluated
    Particle* currentParticle;

    // History of the objective function
    std::vector<double> objective;

    // Modules
    GearModule gear_module;
    TargetSpeedModule target_speed_module;
    AccelBrakeModule accelbrake_module;
    SteeringControlModule steering_module;
    OpponentsModule opponents_module;

    // Number of parameters
    size_t n_parameters;

public:

    // Constructor and destructor
    Controller();
    ~Controller() = default;

    // File-related methods
    void setModelLocation(std::string model_path);
    void loadModel();
    void saveModel();

    // Whether the training algorithm has converged
    bool finishedLearning();

    // Driving methods
    void train(bool is_training);
    void initialize();
    void update(double objective);
    CarControl control(CarState &cs);

    // Getters / setters
    Eigen::VectorXd getLowerBounds();
    Eigen::VectorXd getUpperBounds();
    Eigen::VectorXd getParameters();
    void setParameters(Eigen::VectorXd &parameters);
};


#endif // CONTROLLER_H__
