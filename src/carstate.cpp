/**
    CarState.cpp
    Current car state
    
    @author Antoine Passemiers
    @version 1.0 05/08/2019
*/

#include "CarState.h"


/**
    Constructs car state from a string message received
    from the server.
*/
CarState::CarState(std::string sensors) {
    SimpleParser::parse(sensors, "angle", this->angle);
    SimpleParser::parse(sensors, "curLapTime", this->curLapTime);
    SimpleParser::parse(sensors, "damage", this->damage);
    SimpleParser::parse(sensors, "distFromStart", this->distFromStart);
    SimpleParser::parse(sensors, "distRaced", this->distRaced);
    SimpleParser::parse(sensors, "focus", this->focus, FOCUS_SENSORS_NUM);
    SimpleParser::parse(sensors, "fuel", this->fuel);
    SimpleParser::parse(sensors, "gear", this->gear);
    SimpleParser::parse(sensors, "lastLapTime", this->lastLapTime);
    SimpleParser::parse(sensors, "opponents", this->opponents, OPPONENTS_SENSORS_NUM);
    SimpleParser::parse(sensors, "racePos", this->racePos);
    SimpleParser::parse(sensors, "rpm", this->rpm);
    SimpleParser::parse(sensors, "speedX", this->speedX);
    SimpleParser::parse(sensors, "speedY", this->speedY);
    SimpleParser::parse(sensors, "speedZ", this->speedZ);
    SimpleParser::parse(sensors, "track", this->track, TRACK_SENSORS_NUM);
    SimpleParser::parse(sensors, "trackPos", this->trackPos);
    SimpleParser::parse(sensors, "wheelSpinVel", this->wheelSpinVel, 4);
    SimpleParser::parse(sensors, "z", this->z);
}

/**
    Gets the average wheel speed.

    @return Average wheel speed.
*/
float CarState::getWheelsSpeed() {
    // Compute average wheel angular speed
    float velocity = 0.0;
    for (int i = 0; i < 4; i++) {
        velocity += wheelSpinVel[i];
    }
    velocity /= 4.0;

    // Convert angular speed to ground speed
    double wheel_radius = 0.3325;
    return velocity * wheel_radius * 4.0 * std::pow(M_PI, 2);
}

/**
    Gets the car speed.

    @return Current car speed.
*/
float CarState::getSpeed() {
    // Compute the norm
    float sx = this->speedX;
    float sy = this->speedY;
    float sz = this->speedZ;
    return std::sqrt(sx * sx + sy * sy + sz * sz);
}

/**
    Converts car state to string message.

    @return A string message
*/
string CarState::toString() {
	std::string str;
	str  = SimpleParser::stringify("angle", this->angle);
	str += SimpleParser::stringify("curLapTime", this->curLapTime);
	str += SimpleParser::stringify("damage", this->damage);
	str += SimpleParser::stringify("distFromStart", this->distFromStart);
	str += SimpleParser::stringify("distRaced", this->distRaced);
	str += SimpleParser::stringify("focus", this->focus, FOCUS_SENSORS_NUM);
	str += SimpleParser::stringify("fuel", this->fuel);
	str += SimpleParser::stringify("gear", this->gear);
	str += SimpleParser::stringify("lastLapTime", this->lastLapTime);
	str += SimpleParser::stringify("opponents", this->opponents, OPPONENTS_SENSORS_NUM);
	str += SimpleParser::stringify("racePos", this->racePos);
	str += SimpleParser::stringify("rpm", this->rpm);
	str += SimpleParser::stringify("speedX", this->speedX);
	str += SimpleParser::stringify("speedY", this->speedY);
	str += SimpleParser::stringify("speedZ", this->speedZ);
	str += SimpleParser::stringify("track", this->track, TRACK_SENSORS_NUM);
	str += SimpleParser::stringify("trackPos", this->trackPos);
	str += SimpleParser::stringify("wheelSpinVel", this->wheelSpinVel, 4);
	str += SimpleParser::stringify("z", this->z);
	return str;
}
