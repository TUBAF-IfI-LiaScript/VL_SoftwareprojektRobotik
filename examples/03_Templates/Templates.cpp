#include <iostream>

template<typename T>          // Definition des Typalias
void print (T value){
  std::cout << value << std::endl;
}

int main()
{
  print(5);
  print(10.234);
  print("TU Freiberg");
  return EXIT_SUCCESS;
}
