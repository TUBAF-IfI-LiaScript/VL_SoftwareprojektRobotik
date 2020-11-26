#include "Hall.hpp"
#include "Bedroom.hpp"

Hall::Hall(): RoomType(20, true, "Hall") {}

bool Hall::nextRoom(CleaningRobot* robot){
    bool res = false;
    if (robot->getCleaning()){
      res = executeCleaning();
    }
    changeState(robot, new Bedroom());

    return res;
}
