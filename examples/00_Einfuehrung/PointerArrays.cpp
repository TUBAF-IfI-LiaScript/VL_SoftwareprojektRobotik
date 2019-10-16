#include <iostream>
#include <cstring>

int main() {
  //---------------------------------------------------------------------------
  //Variante I: const char auf dem Heap
  //---------------------------------------------------------------------------
  //warning: ISO C++ forbids converting a string constant to ‘char*’
  //Frage 1: Warum ist das wichtig? Offenbar sieht der C Compiler das ja "nicht
  //         so eng".
  std::cout << typeid("asdfjasfj").name() << std::endl;
  const char* source1 = "ABCDE";
  //Frage 2: Wie kann ich die Inhalte des Speichers arbeiten, Veränderungen
  //         vornehmen?
  //         Ich hatte es zuerst mit einem cast versucht
  //         char* source1 = (char *)"ABCDE";
  //         Bei
  //         strcpy(source1, "xyz");
  //         krachte es dann aber.
  //         printf("%s\n", source1);
  //---------------------------------------------------------------------------
  //Variante II: const char array auf dem Stack
  //---------------------------------------------------------------------------
  char source2[] = "ABCDE";
  strcpy(source2, "xyz");
  printf("%s\n", source2);
  return 0;
}
