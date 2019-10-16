#include <iostream>
#include "logger.hpp"

Logger::Logger()
{
}

Logger::~Logger()
{
}

void Logger::print(const char* message){
  std::cout << message << std::endl;
}
