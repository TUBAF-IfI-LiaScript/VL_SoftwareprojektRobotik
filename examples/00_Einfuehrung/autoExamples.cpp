#include <iostream>
#include <typeinfo>

int main()
{
  auto var1 = 4;
  auto var2 {3.14159};
  auto var3 = "Hallo";
  auto var4 = L"Deutsch Umlaute ÜöÄ";
  auto var5 = new double[10];

  // Datentyp der Variablen ausgeben
  std::cout << typeid(var1).name() << std::endl
      << typeid(var2).name() << std::endl
      << typeid(var3).name() << std::endl
      << typeid(var4).name() << std::endl
      << typeid(var5).name() << std::endl;

  // aufräumen nicht vergessen
  delete[] var5;
}
