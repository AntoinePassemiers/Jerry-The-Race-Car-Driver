/**
    JerryTheRaceCarDriver.h
    Neural network-based driver
    
    @author Antoine Passemiers
    @version 1.0 03/08/2019
*/

#ifndef JerryTheRaceCarDriver_H__
#define JerryTheRaceCarDriver_H__

#include <iostream>
#include <cmath>
#include <string>

#include "SimpleParser.h"

#include "carstate.h"
#include "carcontrol.h"
#include "driver.h"


class JerryTheRaceCarDriver {
public:

    // Current type of race
    typedef enum{ WARMUP, QUALIFYING, RACE, UNKNOWN } tstage;
    tstage stage;

    // Track name
    char trackName[100];

    // Current car state
    CarState cs;

private:
    // Controller for solving driving sub-tasks
    Controller controller;

    // Whether a race restart request has been sent to the server
    bool restart_request_sent = false;

    // Whether the controller is training
    bool is_training = true;

    // Path the file for loading/saving parameter values
    std::string model_path = ".";

    // Current simulation step
    int step = 0;

public:

    // Constructor and destructor
    JerryTheRaceCarDriver();
    ~JerryTheRaceCarDriver() = default;

    // Whether the driver is ready to stop racing
    // If the controller is training, this corresponds
    // to the termination criterion of the optimization algorithm.
    bool readyToShutdown();

    // Initialize rangefinders angles for the client
    void init(float *angles);

    // Restart race
    void restart();

    // Set path to the file where to load/save parameters
    void setModelLocation(std::string path, bool is_training);

    // Evaluate the objective function
    double objective();

    // Drive the car
    std::string drive(std::string sensors);

};

#endif // JerryTheRaceCarDriver_H__
