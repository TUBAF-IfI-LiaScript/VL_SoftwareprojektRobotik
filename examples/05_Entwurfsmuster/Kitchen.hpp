#pragma once
#include "RoomType.hpp"

class Kitchen : public RoomType
{
public:
    Kitchen();
    bool nextRoom(CleaningRobot* robot);
};
