/**
    utils.cpp
    Math functions
    
    @author Antoine Passemiers
    @version 1.0 03/08/2019
*/

#include "utils.h"


/**
    Random sampling of a uniform distributions
    in the range [0, 1].

    @param n Sample size.
    @return Vector containing sample values.
*/
Eigen::VectorXd randUniform(size_t n) {
    Eigen::VectorXd ones = Eigen::VectorXd::Ones(n);
    return (Eigen::VectorXd::Random(n) + ones) / 2.0;
}

/**
    Random sampling of a uniform distributions
    in the range [0, 1].

    @param n Number of rows in the random matrix.
    @param m Number of columns in the random matrix.
    @return Matrix containing sample values.
*/
Eigen::MatrixXd randUniform(size_t n, size_t m) {
    Eigen::MatrixXd ones = Eigen::MatrixXd::Ones(n, m);
    return (Eigen::MatrixXd::Random(n, m) + ones) / 2.0;
}

/**
    Random Gaussian sampling with given mean and standard deviation.

    @param n Sample size.
    @param mu Mean.
    @param std Standard deviation.
    @return Vector containing sample values.
*/
Eigen::VectorXd randGaussian(size_t n, double mu, double std) {
    Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d{ mu, std };
    for (size_t i = 0; i < n; i++) x[i] = d(gen);
    return x;
}

/**
    Random Gaussian sampling with given mean and standard deviation.

    @param n Sample size.
    @param m Number of columns in the random matrix.
    @param mu Mean.
    @param std Standard deviation.
    @return Matrix containing sample values.
*/
Eigen::MatrixXd randGaussian(size_t n, size_t m, double mu, double std) {
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(n, m);
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d{ mu, std };
    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < m; j++) X(i, j) = d(gen);
    }
    return X;
}

/**
    Argmax of a vector.

    @param A vector.
    @return The index of the highest value.
*/
int argmax(Eigen::VectorXd &vec) {
    Eigen::VectorXd::Index maxIndex;
    vec.maxCoeff(&maxIndex);
    return (int)maxIndex;
}

/**
    Inplace tanh function.

    @param X Vector on which to apply the function.
*/
void inplaceTanh(Eigen::VectorXd &X) {
    for (int i = 0; i < X.size(); i++) {
        X[i] = std::tanh(X[i]);
    }
}

/**
    Inplace sigmoid function.

    @param X Vector on which to apply the function.
*/
void inplaceSigmoid(Eigen::VectorXd &X) {
    for (int i = 0; i < X.size(); i++) {
        X[i] = 1.0 / (1.0 + std::exp(-X[i]));
    }
}

/**
    Inplace ReLU function.

    @param X Vector on which to apply the function.
*/
void inplaceReLU(Eigen::VectorXd &X) {
    for (int i = 0; i < X.size(); i++) {
        X[i] = std::max(0.0, X[i]);
    }
}

/**
    Inplace clipping of a vector, ensuring that
    its values stay in the range [0, 1].

    @param X Vector on which to apply the function.
*/
void inplaceClipping(Eigen::VectorXd &X) {
    for (int i = 0; i < X.size(); i++) {
        X[i] = std::max(0.0, std::min(1.0, X[i] + 0.5));
    }
}
