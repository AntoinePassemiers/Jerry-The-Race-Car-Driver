/**
    carcontrol.h
    Car controls
    
    @author Antoine Passemiers
    @version 1.0 05/08/2019
*/

#ifndef CARCONTROL_H__
#define CARCONTROL_H__

#include <iostream>
#include <sstream>
#include <cstring>
#include <cassert>

#include "SimpleParser.h"


class CarControl {
public:
    static const int META_RESTART = 1;

    // Accelerate command [0,1]
    float accel;

    // Brake command [
    float brake;

    // Gear command
    int gear;
    
    // Steering command [-1,1]
    float steer;
    
    // Clutch command [0,1]
    float clutch;

    // focus command [-90,90], i.e. angle of track sensor focus desired by client
    int focus;

    // meta-command
    int meta;

    CarControl() = default;
    CarControl(std::string sensors);
    CarControl(float accel, float brake, int gear, float steer, float clutch, int focus, int meta=0);
    CarControl(float accel, float brake, int gear, float steer, float clutch, int focus=0);
    ~CarControl() = default;

    std::string toString();
    void fromString(std::string sensors);
};

#endif // CARCONTROL_H__
