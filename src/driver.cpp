/**
    driver.cpp
    Interface between PSO algorithm and TORCS
    
    @author Antoine Passemiers
    @version 1.0 01/08/2019
*/

#include "driver.h"


/**
    Constructs the controller, counts the total number of parameters
    and contructs the particle swarm.
*/
Controller::Controller() {
    // Compute total number of parameters in the controller
    this->n_parameters = 0;
    this->n_parameters += accelbrake_module.getNumberOfParameters();
    this->n_parameters += gear_module.getNumberOfParameters();
    this->n_parameters += opponents_module.getNumberOfParameters();
    this->n_parameters += steering_module.getNumberOfParameters();
    this->n_parameters += target_speed_module.getNumberOfParameters();

    // Initialize a PSO with 50 particles and specified
    // values for the hyper-parameters
    this->pso = new PSO(MAXIMIZE, 50, this->n_parameters);
    this->pso->setPhi1(1.87);
    this->pso->setPhi2(1.24);
    this->pso->setInertia(0.85);
    this->initialize();
}

/**
    Checks wether the PSO algorithm has met its termination criterion.
    If the controller is not in training mode, then false is
    returned instead.

    @return Whether the training process has stopped.
*/
bool Controller::finishedLearning() {
    if (this->is_training) {
        bool finished = this->pso->terminationCondition();
        if (finished) {
            std::cout << "PSO termination condition met." << std::endl;
        }
        return finished;
    } else {
        return false; // No training
    }
}

/**
    Specified the controller mode: either training mode
    or normal mode. It the training mode is activated,
    then the parameters are loaded from file.

    @param is_training Whether the training mode is on.
*/
void Controller::train(bool is_training) {
    this->is_training = is_training;
    if (!is_training) {
        this->loadModel();
    }
}

/**
    Setter for the path to the parameter file.

    @param model_path Path to the file used for
        storing/loading controller parameters.
*/
void Controller::setModelLocation(std::string model_path) {
    this->model_path = model_path;
}

/**
    Saves controller parameters to file.
*/
void Controller::saveModel() {
    // Get best particle position
    Eigen::VectorXd parameters = this->pso->getBestPosition();

    // Stores parameters in a text file
    std::ofstream file(this->model_path);
    if (file.is_open()) {
        for (int i = 0; i < parameters.size(); i++) {
            file << parameters[i] << " ";
        }
    } else {
        std::cout << "Cannot save file " << this->model_path << std::endl;
        throw "Cannot save file " + this->model_path;
    }
}

/**
    Loads controller parameters from file.
*/
void Controller::loadModel() {
    // Allocate space for storing parameters
    size_t n = this->pso->n_dim;
    Eigen::VectorXd parameters = Eigen::VectorXd::Zero(n);

    // Read text file
    std::ifstream file(this->model_path);
    if (file.is_open()) {
        double num;
        for (size_t i = 0; i < n; i++) {
            file >> num;
            parameters[i] = num;
        }
    } else {
        throw "Cannot load file " + this->model_path;
    }

    // Updates the parameters of all controller modules
    this->setParameters(parameters);
}

/**
    Initializes the controller and particle swarm optimizer.
*/
void Controller::initialize() {
    // Initializes empty history of evaluations of
    // the objective function
    this->objective = std::vector<double>();

    // Retrieves the concatenation of lower and upper
    // bounds of the module parameters
    Eigen::VectorXd lbs = this->getLowerBounds();
    Eigen::VectorXd ubs = this->getUpperBounds();

    // Initializes the particle swarm
    for (size_t i = 0; i < this->pso->n_particles; i++) {
        this->pso->swarm[i]->initialize(lbs, ubs);
    }

    // Gets the first particle to be evaluated
    this->currentParticle = this->pso->next();

    // Updates module parameters
    if (this->is_training) {
        this->setParameters(this->currentParticle->getCurrentPosition());
    } else {
        this->setParameters(this->pso->getBestPosition());
    }
}

/**
    Drives the car by sending the controls outputed
    by the different modules.

    @param cs Current car state.
    @return Car controls.
*/
CarControl Controller::control(CarState &cs) {
    CarControl cc;
    cc.clutch = 0.0; // Clutch is not considered in the model

    // Get module outputs based on sensory data
    int gear = this->gear_module.control(cs);
    double target_speed = this->target_speed_module.control(cs);
    double accelbrake = this->accelbrake_module.control(cs, target_speed);
    double steer = this->steering_module.control(cs);

    // Apply adjustments on the outputs based on opponent sensors
    this->opponents_module.control(cs, steer, accelbrake);

    // Acceleration and brake are set by the same control variable
    // to avoid nonsense outputs
    if (accelbrake > 0.5) { // Acceleration case
        cc.brake = 0.0;
        cc.accel = (accelbrake - 0.5) * 2.0;
    } else { // Brake case
        cc.accel = 0.0;
        cc.brake = 1.0 - accelbrake * 2.0;
    }
    // Sets gear and steering
    cc.gear = gear;
    cc.steer = steer;
    return cc;
}

/**
    Updates the controller and the particle swarm optimizer.

    @param objective Evaluation of the objective function.
*/
void Controller::update(double objective) {
    this->objective.push_back(objective);

    // Display the history of evaluations of the objective function
    if (this->objective.size() % 50 == 0) {
        for (size_t i = 0; i < this->objective.size(); i++) {
            std::cout << this->objective.at(i) << ", ";
        }
        std::cout << std::endl;
    }

    // Updates PSO and module parameters
    if (this->is_training) {
        // Sets the evaluation of the particle for its current position
        this->currentParticle->setEvaluation(objective);

        // Notify the PSO that it should check whether the new solution
        // is the new global best solution
        this->pso->update();

        // Gets the next particle to be evaluated
        this->currentParticle = this->pso->next();

        // Updates module parameters based on the new particle's position
        this->setParameters(this->currentParticle->getCurrentPosition());

        // Save module parameters
        this->saveModel();
    }
}

/**
    Concatenates lowerbounds of modules parameters
    and returns them as a single vector.

    @return Lower bounds on the modules parameters.
*/
Eigen::VectorXd Controller::getLowerBounds() {
    // Get lower bounds of accelbrake module parameters
    Eigen::VectorXd lbs = Eigen::VectorXd::Zero(this->n_parameters);
    int j = 0;
    int n = this->accelbrake_module.getNumberOfParameters();
    lbs.segment(j, n) = this->accelbrake_module.getLowerBounds();

    // Get lower bounds of gear selection module parameters
    j += n;
    n = this->gear_module.getNumberOfParameters();
    lbs.segment(j, n) = this->gear_module.getLowerBounds();

    // Get lower bounds of opponents module parameters
    j += n;
    n = this->opponents_module.getNumberOfParameters();
    lbs.segment(j, n) = this->opponents_module.getLowerBounds();

    // Get lower bounds of steering module parameters
    j += n;
    n = this->steering_module.getNumberOfParameters();
    lbs.segment(j, n) = this->steering_module.getLowerBounds();

    // Get lower bounds of target speed module parameters
    j += n;
    n = this->target_speed_module.getNumberOfParameters();
    lbs.segment(j, n) = this->target_speed_module.getLowerBounds();
    return lbs;
}

/**
    Concatenates upper bounds of modules parameters
    and returns them as a single vector.

    @return Upper bounds on the modules parameters.
*/
Eigen::VectorXd Controller::getUpperBounds() {
    // Get upper bounds of accelbrake module parameters
    Eigen::VectorXd ubs = Eigen::VectorXd::Zero(this->n_parameters);
    int j = 0;
    int n = this->accelbrake_module.getNumberOfParameters();
    ubs.segment(j, n) = this->accelbrake_module.getUpperBounds();

    // Get upper bounds of gear selection module parameters
    j += n;
    n = this->gear_module.getNumberOfParameters();
    ubs.segment(j, n) = this->gear_module.getUpperBounds();

    // Get upper bounds of opponents module parameters
    j += n;
    n = this->opponents_module.getNumberOfParameters();
    ubs.segment(j, n) = this->opponents_module.getUpperBounds();

    // Get upper bounds of steering module parameters
    j += n;
    n = this->steering_module.getNumberOfParameters();
    ubs.segment(j, n) = this->steering_module.getUpperBounds();

    // Get upper bounds of target speed module parameters
    j += n;
    n = this->target_speed_module.getNumberOfParameters();
    ubs.segment(j, n) = this->target_speed_module.getUpperBounds();
    return ubs;
}

/**
    Concatenates modules parameters and return them
    as a single vector.

    @return Current values of modules parameters.
*/
Eigen::VectorXd Controller::getParameters() {
    // Get accebrake module parameters
    Eigen::VectorXd parameters = Eigen::VectorXd::Zero(this->n_parameters);
    int j = 0;
    int n = this->accelbrake_module.getNumberOfParameters();
    parameters.segment(j, n) = this->accelbrake_module.getParameters();

    // Get gear module parameters
    j += n;
    n = this->gear_module.getNumberOfParameters();
    parameters.segment(j, n) = this->gear_module.getParameters();

    // Get opponents module parameters
    j += n;
    n = this->opponents_module.getNumberOfParameters();
    parameters.segment(j, n) = this->opponents_module.getParameters();

    // Get steering module parameters
    j += n;
    n = this->steering_module.getNumberOfParameters();
    parameters.segment(j, n) = this->steering_module.getParameters();

    // Get target speed module parameters
    j += n;
    n = this->target_speed_module.getNumberOfParameters();
    parameters.segment(j, n) = this->target_speed_module.getParameters();
    return parameters;
}

/**
    Sets current values of modules parameters.

    @param Current values of modules parameters,
        stored as a single vector.
*/
void Controller::setParameters(Eigen::VectorXd &parameters) {
    // Set accelbrake module parameters
    int j = 0;
    int n = this->accelbrake_module.getNumberOfParameters();
    Eigen::VectorXd segment = parameters.segment(j, n);
    this->accelbrake_module.setParameters(segment);
    j += n;
    n = this->gear_module.getNumberOfParameters();
    segment = parameters.segment(j, n);
    this->gear_module.setParameters(segment);

    // Set opponents module parameters
    j += n;
    n = this->opponents_module.getNumberOfParameters();
    segment = parameters.segment(j, n);
    this->opponents_module.setParameters(segment);

    // Set steering module parameters
    j += n;
    n = this->steering_module.getNumberOfParameters();
    segment = parameters.segment(j, n);
    this->steering_module.setParameters(segment);

    // Set target speed module parameters
    j += n;
    n = this->target_speed_module.getNumberOfParameters();
    segment = parameters.segment(j, n);
    this->target_speed_module.setParameters(segment);
}
