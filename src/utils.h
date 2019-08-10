/**
    utils.h
    Math functions
    
    @author Antoine Passemiers
    @version 1.0 03/08/2019
*/

#ifndef UTILS_H__
#define UTILS_H__

#include <Eigen/Core>
#include <cmath>
#include <random>

// Types of activation functions
#define ACTIVATION_SIGMOID  0
#define ACTIVATION_TANH     1
#define ACTIVATION_RELU     2
#define ACTIVATION_CLIPPING 3

// Random uniform generation of vectors
Eigen::VectorXd randUniform(size_t n);

// Random uniform generation of matrices
Eigen::MatrixXd randUniform(size_t n, size_t m);

// Random gaussian generation of vectors
Eigen::VectorXd randGaussian(size_t n, double mu, double std);

// Random gaussian generation of matrices
Eigen::MatrixXd randGaussian(size_t n, size_t m, double mu, double std);

// Argmax of an Eigen vector
int argmax(Eigen::VectorXd &vec);

// MLP activation functions
void inplaceSigmoid(Eigen::VectorXd &X);
void inplaceTanh(Eigen::VectorXd &X);
void inplaceReLU(Eigen::VectorXd &X);
void inplaceClipping(Eigen::VectorXd &X);


#endif // UTILS_H__
