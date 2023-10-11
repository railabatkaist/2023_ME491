#include <iostream>
#include "student_id.hpp"

int main() {
  State state1, state2;

  std::cout << "optimal value for the state: " << getOptimalValue(state1) << std::endl;
  std::cout << "optimal action sequence for the state: ";
  printOptimalActionSequence(state2);

  return 0;
}
