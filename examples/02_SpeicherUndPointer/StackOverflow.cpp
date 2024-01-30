//g++ StackOverflow.cpp

#include <iostream>

int recursion(int a)
{
        return recursion(a+1);
}

int main()
{
        std::cout << "Stack Overflow about to occur!\n";
        double array[1024*1024];
        // recursion(0);
        return EXIT_FAILURE;
}
