/**
    JerryTheRaceCarDriver.cpp
    Neural network-based driver
    
    @author Antoine Passemiers
    @version 1.0 03/08/2019
*/

#include "JerryTheRaceCarDriver.h"


/**
    Driver constructor. Initializes the controller.
*/
JerryTheRaceCarDriver::JerryTheRaceCarDriver() {
    this->controller = Controller();
    this->restart_request_sent = false;
    this->step = 0;
}

/**
    Defines rangefinders angles for the client.

    @param angles Array of angles to be filled
        in order for the client to communicate
        them to the TORCS server.
*/
void JerryTheRaceCarDriver::init(float* angles) {
    for (int i = 0; i < 19; i++) {
        angles[i] = -90 + i * 10;
    }
}

/**
    Sets path to the file where to load/save parameters,
    and specifies whether the controller should be trained
    or not.

    @param path Location of the parameter file.
    @param is_training Whether to train the controller.
*/
void JerryTheRaceCarDriver::setModelLocation(std::string path, bool is_training) {
    this->model_path = path;
    this->is_training = is_training;
    this->controller.setModelLocation(path);
    this->controller.train(is_training);
}

/**
    Evaluates the objective function, defined as the
    total distance raced from the beginning of the race,
    minus a penalty term proportional to the total
    damage taken by the car.

    @return The evaluation of the objective function.
*/
double JerryTheRaceCarDriver::objective() {
    double obj = this->cs.distRaced;
    obj -= this->cs.damage * 2.0;
    cs.distRaced = 0.0;
    return obj;
}

/**
    Restarts the race and evaluates the objective function
    based on the car state at the end of the race that
    just finished. This method is called by the client
    once the server has received the start request.
*/
void JerryTheRaceCarDriver::restart() {
    // The server has received the request
    this->restart_request_sent = false;

    // Reset the counter of simulation steps
    this->step = 0;

    // Evaluate the objective function
    double obj = this->objective();

    if (std::abs(obj) > 10) { // Avoid extra evaluations due to client-server latency
        // Display information
        if (cs.lastLapTime != 0.0) {
            std::cout << "Last lap time: " << cs.lastLapTime << std::endl;
            std::cout << "Race rank: " << cs.racePos << std::endl;
        }
        std::cout << "Value of the objective function: " << obj << std::endl;

        // Updates the controller and propagates the evaluation
        // of the objective function back to the optimization algorithm
        this->controller.update(obj);
    }
}

/**
    Drives the car.

    @param sensors A string to be parsed, containing all the information
        about current state of the car.
    @return The car controls to be sent to the server.
*/
std::string JerryTheRaceCarDriver::drive(std::string sensors) {
    // Transfers car state to the controller and retrieves car controls
    CarState cs(sensors);
    CarControl cc = this->controller.control(cs);

    // Stores current car state for future evaluation of
    // the objective function
    this->cs = cs;

    // No need to go further in the case of a race restart
    if (this->restart_request_sent) return cc.toString();

    // Increment the number of simulation steps
    this->step++;

    if (this->is_training) {
        // If timeout occurs, if the car takes too much damage, if the car
        // runs out of fuel or if the car is not making any progression
        // from the start line, then restart the race.
        if ((cs.curLapTime > 300) || (cs.damage > 1000) || (cs.fuel < 0.05) \
                || ((this->step > 100) && (cs.distRaced < 0))) {
            this->restart_request_sent = true;
            cc.meta = 1; // Race restart request
        }
    }
    return cc.toString();
}

/**
    Whether the driver is ready to stop racing.
    When the driver is training, this corresponds to the termination
    criterion of the optimization algorithm.

    @return Whether to stop racing.
*/
bool JerryTheRaceCarDriver::readyToShutdown() {
    if (this->is_training) {
        return this->controller.finishedLearning();
    } else{
        return false;
    }
}
