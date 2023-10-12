#include <iostream>
#include "student_id.hpp"

int main() {
  Eigen::Vector<int, 12> state1, state2;

  std::cout << "optimal value for the state: " << getOptimalValue(state1) << std::endl;
  std::cout << "optimal action sequence for the state: ";
  printOptimalAction(state2);

  return 0;
}
