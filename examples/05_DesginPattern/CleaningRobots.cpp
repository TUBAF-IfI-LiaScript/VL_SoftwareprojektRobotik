#include "CleaningRobot.hpp"
#include "Kitchen.hpp"
#include "Hall.hpp"

CleaningRobot::CleaningRobot(): m_CurrentState(new Kitchen),
                                 homePosition(new Hall),
                                 isCleaning(false){}

bool CleaningRobot::nextRoom() {
    return m_CurrentState->nextRoom(this);
}

bool CleaningRobot::returnHome(){
    return m_CurrentState->returnHome(this);
}

void CleaningRobot::setCleaning(bool cleaning){
    isCleaning = cleaning;
}

bool CleaningRobot::getCleaning(){
    return isCleaning;
}

void CleaningRobot::setCurrentState(RoomType * currentState) {
    if (m_CurrentState) {
        delete m_CurrentState;
        m_CurrentState = NULL;
    }
    m_CurrentState = currentState;
}
