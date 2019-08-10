/**
    mlp.h
    Multi-layer perceptrons
    
    @author Antoine Passemiers
    @version 1.0 01/08/2019
*/

#ifndef MLP_H__
#define MLP_H__

#include <Eigen/Core>
#include <cassert>
#include <vector>
#include <iostream>

#include "utils.h"


class MLP {
private:

    // Number of inputs
    size_t n_inputs;

    // Add biases in linear layers
    bool use_bias;

    // Layers inputs/outputs
    std::vector<Eigen::VectorXd> h;

    // Layers parameters
    std::vector<Eigen::MatrixXd> A;
    std::vector<Eigen::VectorXd> b;

    // Activation functions
    std::vector<short> activations;

public:
    // Constructors and destructor
    MLP(size_t n_inputs);
    MLP(size_t n_inputs, bool use_bias);
    ~MLP() = default;

    void addFullyConnectedLayer(size_t n_inputs, size_t n_outputs);
    void addActivation(short activation);

    // Network input and output.
    // These accessors allow the controller
    // to access values one-by-one (e.g. accelCmd).
    double& in(int i);
    double out(int i);
    Eigen::VectorXd& out();

    // Number of parameters in the network
    size_t getNumberOfParameters();

    // Set parameters values
    void initWeights();
    void setWeights(const Eigen::VectorXd &weights);
    Eigen::VectorXd getWeights();

    // Refresh the output values
    void forward();

};

#endif // MLP_H__
