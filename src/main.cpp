#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "student_id.hpp"

int main() {
  if(getOptimalValue(Eigen::Matrix3d::Zero())>-1) std::cout<<"loaded"<<std::endl;

  return 0;
}
