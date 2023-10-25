#include<iostream>

void log (const char* message)
{
    std::cout << message << std::endl;
}

int main(void)
{
    log("Hello World!");
    return EXIT_SUCCESS;
}