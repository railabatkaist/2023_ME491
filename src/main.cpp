#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "student_id.hpp"

int main() {
  Eigen::Matrix3d state;
  state<<1,0,0,0,-1,0,0,0,0;
  std::cout<<"optimal value for state: "<<getOptimalValue(state)<<std::endl;
  return 0;
}
