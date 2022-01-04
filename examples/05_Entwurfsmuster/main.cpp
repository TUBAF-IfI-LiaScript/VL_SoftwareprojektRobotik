#include <iostream>
#include "CleaningRobot.hpp"
#include "RoomType.hpp"

const int roomNumber = 3;

std::ostream& operator<<(std::ostream& os, const CleaningRobot& robot)
{
    std::string name;
    bool light;
    int speed;
    (robot.m_CurrentState)->getStateParameters(speed, light, name);
    os << "go to "<< name << "(speed=" << speed << ",light=" << light
       << ")"<<std::endl;
    return os;
}

int main(){
    CleaningRobot my_robot;
    // Starting cleaning phase
    my_robot.setCleaning(true);
    for (int i=0; i<roomNumber; i++){
      std::cout << my_robot;
      my_robot.nextRoom();
    }
    my_robot.setCleaning(false);
    my_robot.returnHome();
    std::cout << my_robot;
    return EXIT_SUCCESS;
}
