#include <iostream>

int calc(int factor1, int factor2){
  return factor1 * factor2;
}

int main()
{
  int num1 {0x11};
  int num2 {0x22};
  int result {0};
  result = calc(num1, num2);
  std::cout << result << std::endl;
  return EXIT_SUCCESS;
}
