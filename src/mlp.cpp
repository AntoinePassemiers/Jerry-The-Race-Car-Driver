/**
    mlp.cpp
    Multi-layer perceptrons
    
    @author Antoine Passemiers
    @version 1.0 01/08/2019
*/

#include "mlp.h"


/**
    Constructor with implicit biases.

    @param n_inputs: Number of inputs.
*/
MLP::MLP(size_t n_inputs) : MLP(n_inputs, true) {}

/**
    Constructor.

    @param n_inputs: Number of inputs.
    @param use_bias: Whether to add biases.
*/
MLP::MLP(size_t n_inputs, bool use_bias) {
    this->n_inputs = n_inputs;
    this->use_bias = use_bias;

    // Initialize layer vectors
    this->A = std::vector<Eigen::MatrixXd>();
    this->b = std::vector<Eigen::VectorXd>();
    this->h = std::vector<Eigen::VectorXd>();
    this->activations = std::vector<short>();

    // Add vector for storing input values
    this->h.push_back(Eigen::VectorXd::Zero(n_inputs));
}

/**
    Accesses an input of the network.

    @param Index of the input value.
    @return Reference to the value.
*/
double& MLP::in(int i) {
    return this->h[0][i];
}

/**
    Accesses an output of the network.

    @param Index of the output value.
    @return Output value.
*/
double MLP::out(int i) {
    return this->h[this->h.size() - 1][i];
}

/**
    Returns the outputs of the network
    as a single vector.
*/
Eigen::VectorXd& MLP::out() {
    return this->h[this->h.size() - 1];
}

/**
    Adds a fully-connected layer to the network.

    @param n_inputs Number of input neurons.
    @param n_outputs Number of output neurons.
*/
void MLP::addFullyConnectedLayer(size_t n_inputs, size_t n_outputs) {
    this->A.push_back(Eigen::MatrixXd::Zero(n_inputs, n_outputs));
    if (this->use_bias) { // Add biases if required
        this->b.push_back(Eigen::VectorXd::Zero(n_outputs));
    }
    this->h.push_back(Eigen::VectorXd(n_outputs));
}

/**
    Adds an activation function to the network.

    @param activation Enum representing the activation function.
*/
void MLP::addActivation(short activation) {
    this->activations.push_back(activation);
}

/**
    Returns the number of parameters in the network,
    namely the number of weights, plus the number of biases
    if required.

    @return Number of parameters.
*/
size_t MLP::getNumberOfParameters() {
    int n = 0;
    for (size_t i = 0; i < this->A.size(); i++) { // For each layer
        n += this->A[i].cols() * this->A[i].rows();
        // Add the number of biases if present in the network
        if (this->use_bias) {
            n += this->b[i].size();
        }
    }
    return n;
}

/**
    Initializes parameters values.
*/
void MLP::initWeights() {
    for (size_t i = 0; i < this->A.size(); i++) { // For each layer
        // Compute standard deviation for the Xavier initialization
        size_t n_in = this->A[i].rows();
        size_t n_out = this->A[i].cols();
        double variance = 2.0 / (n_in + n_out);

        // Sample a Gaussian distribution
        this->A[i] = randGaussian(n_in, n_out, 0.0, std::sqrt(variance));

        // Initialize biases if present in the network
        // Biases are not zero-initialized since the network
        // is being optimized with a particle swarm.
        if (this->use_bias) {
            variance = 1.0 / n_out;
            this->b[i] = randGaussian(n_out, 0.0, std::sqrt(variance));
        }
    }
}

/**
    Gets the concatenation of all network parameters.

    @return Network parameters.
*/
Eigen::VectorXd MLP::getWeights() {
    // Allocate space for all the parameters
    size_t n = this->getNumberOfParameters();
    Eigen::VectorXd weights = Eigen::VectorXd::Zero(n);

    int j = 0;
    for (size_t k = 0; k < this->A.size(); k++) { // For each layer
        Eigen::MatrixXd &A = this->A[k];

        // Store matrix A line by line in the concatenated vector
        size_t n_inputs = A.rows();
        size_t n_outputs = A.cols();
        for (size_t i = 0; i < n_inputs; i++) {
            weights.segment(j + i * n_outputs, n_outputs) = A.row(i);
        }
        j += n_inputs * n_outputs;

        // Store biases (if present) in the concatenated vector
        if (this->use_bias) {
            weights.segment(j, n_outputs) = this->b[k];
            j += n_outputs;
        }
    }

    assert(j == weights.size());
    return weights;
}

/**
    Sets the parameters values.

    @param weights Parameter values, provided as a single vector.
*/
void MLP::setWeights(const Eigen::VectorXd &weights) {

    int j = 0;

    for (size_t k = 0; k < this->A.size(); k++) { // For each layer
        // Store values line-by-line in matrix A
        size_t n_inputs = this->A[k].rows();
        size_t n_outputs = this->A[k].cols();
        Eigen::VectorXd _A = weights.segment(j, n_inputs * n_outputs);
        for (size_t i = 0; i < n_inputs; i++) {
            this->A[k].row(i) = _A.segment(i * n_outputs, n_outputs);
        }
        j += n_inputs * n_outputs;

        // If present, store the values of the biases
        if (this->use_bias) {
            this->b[k] = weights.segment(j, n_outputs);
            j += n_outputs;
        }
    }

    assert(j == weights.size());
}

/**
    Computes the outputs of the network based on the input values.
*/
void MLP::forward() {
    size_t n_layers = this->A.size();
    for (size_t k = 0; k < n_layers; k++) { // For each layer
        // Linear operation
        this->h[k + 1] = this->A[k].transpose() * this->h[k];
        if (this->use_bias) { // Add biases if present in the network
            this->h[k + 1] += this->b[k];
        }

        // Apply activation function
        if (k < this->activations.size()) {
            switch (this->activations[k]) {
                case ACTIVATION_SIGMOID:
                    inplaceSigmoid(this->h[k + 1]);
                    break;
                case ACTIVATION_TANH:
                    inplaceTanh(this->h[k + 1]);
                    break;
                case ACTIVATION_RELU:
                    inplaceReLU(this->h[k + 1]);
                    break;
                case ACTIVATION_CLIPPING:
                    inplaceClipping(this->h[k + 1]);
                    break;
                default:
                    inplaceSigmoid(this->h[k + 1]);
            }
        }
    }
}
