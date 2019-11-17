#pragma once
#include "CleaningRobot.hpp"
#include <iostream>

class RoomType
{
protected:
    int speed;   // cm/s
    bool warningLightOn;
    std::string name;
public:
    RoomType(int speed, bool warningLightOn, std::string name);
    void getStateParameters(int &speed, bool& light, std::string& name);
    virtual bool nextRoom(CleaningRobot* robot) = 0;
    bool returnHome(CleaningRobot* robot);
    bool executeCleaning();
protected:
    void changeState(CleaningRobot* robot, RoomType* newState);
};
