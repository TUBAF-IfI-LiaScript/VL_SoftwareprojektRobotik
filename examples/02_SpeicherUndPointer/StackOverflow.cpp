//g++ StackOverflow.cpp -fstack-check

#include <iostream>

int main(void)
{
    double array[1024*1024];
    //double* array = new double [1024*1024];
    //std::cout << array[1000] << std::endl;
    std::cout << sizeof(double) << std::endl;
    return EXIT_SUCCESS;
}
