#include "RoomType.hpp"

RoomType::RoomType(int speed,
                   bool warningLightOn,
                   std::string name)
                   : speed(speed), warningLightOn(warningLightOn), name(name){}

bool RoomType::returnHome(CleaningRobot* robot){
    robot->setCurrentState((RoomType*)robot->homePosition);
    return true;
}

bool RoomType::executeCleaning(){
    std::cout << "    ... execute cleaning in " << name << std::endl;
    return true;
}

void RoomType::getStateParameters(int &speed, bool& light, std::string& name){
    speed = this->speed;
    light = this->warningLightOn;
    name = this->name;
}

void RoomType::changeState(CleaningRobot* robot, RoomType* newState){
    robot->setCurrentState(newState);
}
