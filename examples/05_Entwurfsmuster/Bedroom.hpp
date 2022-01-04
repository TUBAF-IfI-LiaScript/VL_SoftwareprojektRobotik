#pragma once
#include "RoomType.hpp"

class Bedroom : public RoomType
{
public:
    Bedroom();
    bool nextRoom(CleaningRobot* robot);
};
