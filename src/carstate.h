/**
    CarState.h
    Current car state
    
    @author Antoine Passemiers
    @version 1.0 05/08/2019
*/

#ifndef CARSTATE_H__
#define CARSTATE_H__

#define FOCUS_SENSORS_NUM 5
#define TRACK_SENSORS_NUM 19
#define OPPONENTS_SENSORS_NUM 36

#include <cassert>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <sstream>

#include "SimpleParser.h"


class CarState {
public:
        // Angle with the middle track line
        float angle;

        // Current lap time
        float curLapTime;

        // Total damage taken by the car
        float damage;

        // Distance remaining before reaching
        // the start line
        float distFromStart;

        // Distance raced from the beginning of the race
        float distRaced;

        // Sensors controlled by the user
        float focus[FOCUS_SENSORS_NUM];

        // Level of fuel
        float fuel;

        // Gear selection
        int gear;

        // Last lap time
        float lastLapTime;

        // Opponents sensors
        float opponents[OPPONENTS_SENSORS_NUM];

        // Current race rank
        int racePos;

        // Rotation per minute
        int rpm;

        // Velocity
        float speedX;
        float speedY;
        float speedZ;

        // Track borders sensors
        float track[TRACK_SENSORS_NUM];

        // Position of the car from the middle track line
        float trackPos;

        // Wheel speeds
        float wheelSpinVel[4];

        // Car elevation
        float z;
	
        // Constructors and destructor
		CarState() = default;
        CarState(std::string sensors);
        ~CarState() = default;

        // Get average wheel speed
        float getWheelsSpeed();

        // Get car speed
        float getSpeed();

        // Convert to string
        string toString();
};

#endif // CARSTATE_H__
