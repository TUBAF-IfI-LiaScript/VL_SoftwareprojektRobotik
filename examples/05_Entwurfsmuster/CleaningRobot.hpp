#pragma once
#include <iostream>

class RoomType;

class CleaningRobot
{
private:
    RoomType * m_CurrentState;
    bool isCleaning;
public:
    RoomType * homePosition;
public:
    CleaningRobot();
    bool nextRoom();
    bool returnHome();
    void setCurrentState(RoomType * currentState);
    void setCleaning(bool);
    bool getCleaning();
    friend std::ostream& operator<<(std::ostream& os, const CleaningRobot& robot);
};
