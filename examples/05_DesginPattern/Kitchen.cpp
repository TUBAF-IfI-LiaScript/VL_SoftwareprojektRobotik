#include "Hall.hpp"
#include "Kitchen.hpp"

Kitchen::Kitchen(): RoomType(10, true, "Kitchen") {}

bool Kitchen::nextRoom(CleaningRobot* robot){
    bool res = false;
    if (robot->getCleaning()){
      res = executeCleaning();
    }
    changeState(robot, new Hall());

    return res;
}
