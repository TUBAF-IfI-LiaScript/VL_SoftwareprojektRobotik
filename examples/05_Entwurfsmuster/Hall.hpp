#pragma once
#include "RoomType.hpp"

class Hall : public RoomType
{
public:
    Hall();
    bool nextRoom(CleaningRobot* robot);
};
