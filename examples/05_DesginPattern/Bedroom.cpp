#include "Kitchen.hpp"
#include "Bedroom.hpp"

Bedroom::Bedroom(): RoomType(20, false, "Bedroom") {}

bool Bedroom::nextRoom(CleaningRobot* robot){
    bool res = false;
    if (robot->getCleaning()){
      res = executeCleaning();
    }
    changeState(robot, new Kitchen());

    return res;
}
