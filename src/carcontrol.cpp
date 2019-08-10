/**
    carcontrol.cpp
    Car controls
    
    @author Antoine Passemiers
    @version 1.0 05/08/2019
*/

#include "carcontrol.h"


CarControl::CarControl(float accel, float brake, int gear, float steer, float clutch, int focus, int meta) :
    accel(accel), brake(brake), gear(gear), steer(steer), clutch(clutch), focus(focus), meta(meta) {}

CarControl::CarControl(float accel, float brake, int gear, float steer, float clutch, int focus) :
    CarControl(accel, brake, gear, steer, clutch, focus, 0) {}

CarControl::CarControl(string sensors) {
    this->fromString(sensors);
}

std::string CarControl::toString() {
    std::string str;
    str  = SimpleParser::stringify("accel", accel);
    str += SimpleParser::stringify("brake", brake);
    str += SimpleParser::stringify("gear",  gear);
    str += SimpleParser::stringify("steer", steer);
    str += SimpleParser::stringify("clutch", clutch);
    str += SimpleParser::stringify("focus",  focus);
    str += SimpleParser::stringify("meta", meta);
    return str;    
}

void  CarControl::fromString(string sensors) {
    if (SimpleParser::parse(sensors, "accel", accel)==false)
        accel=0.0;
    if (SimpleParser::parse(sensors, "brake", brake)==false)
        brake=0.0;
    if (SimpleParser::parse(sensors, "gear",  gear)==false)
        gear=1;
    if (SimpleParser::parse(sensors, "steer", steer)==false)
        steer=0.0;
    if (SimpleParser::parse(sensors, "clutch", clutch)==false)
            clutch=0.0;
    if (SimpleParser::parse(sensors, "meta", meta)==false)
        meta=0;
    if (SimpleParser::parse(sensors, "focus", focus)==false) //ML
        focus=0; //ML
    if (focus < -90 || focus > 90)//ML What to do with focus requests out of allowed range?
        focus=360;//ML A value of 360 is used for not requesting focus readings; -1 is returned as focus reading to the client
}
