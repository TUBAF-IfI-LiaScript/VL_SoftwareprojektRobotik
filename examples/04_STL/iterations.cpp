#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <stdlib.h>     /* srand, rand */

int main()
{
  srand (time(NULL));

  std::vector<int> vec(100000000);
  std::generate(vec.begin(), vec.end(), std::rand);

  // Iteration mittels Indexvariable
  auto begin= std::chrono::steady_clock::now();
  for (unsigned int i = 0; i < vec.size(); i++){
     vec[i] += rand() % 10 + 1;
  }
  std::chrono::duration<double> last=  std::chrono::steady_clock::now() - begin;
  std::cout << "time: " << last.count() << std::endl;

  // Anwendung des Iteratorkonzepts
  begin= std::chrono::steady_clock::now();
  for (std::vector<int>::iterator itr = vec.begin(); itr!=vec.end();  ++itr){
     *itr += rand() % 10 + 1;
  }
  last=  std::chrono::steady_clock::now() - begin;
  std::cout << "time: " << last.count() << std::endl;

  begin= std::chrono::steady_clock::now();
  for (auto it: vec){               // neu in C++11
     it +=rand() % 10 + 1;
  }
  last=  std::chrono::steady_clock::now() - begin;
  std::cout << "time: " << last.count() << std::endl;

  std::cout << vec.at(0) << ", " << vec.at(1) << " ..." << std::endl;
}
