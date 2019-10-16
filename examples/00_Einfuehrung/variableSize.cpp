#include <iostream>
#include <typeinfo>

int main()
{
	std::cout<<typeid(int).name() << " - "<<  sizeof(int) << " Byte \n";
  std::cout<<typeid(int32_t).name()  << " - "<<  sizeof(int32_t) << " Byte \n";
  std::cout<<typeid(int_least32_t).name() << " - "<<  sizeof(int_least32_t) << " Byte \n";
  std::cout<<typeid(int_fast32_t).name() << " - "<<  sizeof(int_fast32_t) << " Byte \n";
  std::cout<< "Und jetzt noch size_t \n";  
	std::cout<<typeid(size_t).name() << " - "<<  sizeof(size_t) << " Byte \n";
}
