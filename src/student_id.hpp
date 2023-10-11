#include <iostream>
#include <vector>
#include <exception>
#include <Eigen/Core>
#include <Eigen/Dense>

class State {
 public:
  State() {
    init();
  }

  void init() {
    state_.setZero();
    boxes_.resize(4);
    boxes_[0].push_back(0); boxes_[0].push_back(2); boxes_[0].push_back(6); boxes_[0].push_back(8);  // left upper box
    boxes_[1].push_back(1); boxes_[1].push_back(3); boxes_[1].push_back(8); boxes_[1].push_back(10);  // right upper box
    boxes_[2].push_back(2); boxes_[2].push_back(4); boxes_[2].push_back(7); boxes_[2].push_back(9);  // left lower box
    boxes_[3].push_back(3); boxes_[3].push_back(5); boxes_[3].push_back(9); boxes_[3].push_back(11);  // right lower box
  }

  double drawLine(int idx) {
    // draw the line and return the corresponding reward.
    if ((idx >= 12) || (idx < 0)) {
      std::string msg = "The index is out of range! index: " + std::to_string(idx);
      throw std::logic_error(msg);
    }
    if (state_(idx) == 1) {
      std::string msg = "The line is already drawn! index: " + std::to_string(idx);
      throw std::logic_error(msg);
    }

    state_(idx) = 1;

    return computeReward(idx);
  }

  double computeReward(int idx) {
    int boxGroup;
    for (int i=0; i<4; i++) {
      if (std::find(boxes_[i].begin(), boxes_[i].end(), idx) != boxes_[i].end()) {
        boxGroup = i;
        break;
      }
    }

    int count = 0;
    for (int n=0; n<4; n++) {
      count += state_[boxes_[boxGroup][n]];
    }
    if (count == 4)
      return 1.;
    else
      return 0.;
  }

 private:
  // The first 6 elements represent horizontal line state.
  // The last 6 elements represent vertical line state.
  // 1 means the line is drawn, and 0 means the opposite.
  Eigen::Vector<int, 12> state_;
  // boxes_ represents indices of the state to form each box.
  std::vector<std::vector<int>> boxes_;
};

/// DO NOT CHANGE THE NAME AND FORMAT OF THIS FUNCTION
double getOptimalValue(State state){
  // return the optimal value of the given state
  /// TODO
  return 0.0; // return optimal value
}

/// DO NOT CHANGE THE NAME AND FORMAT OF THIS FUNCTION
void printOptimalActionSequence(State state){
  // print the optimal action sequence from the given state until the game ends.
  // the action should be represented as a state index.
  // also, the opponents' action should be printed in parentheses.
  // ex) 5-(1)-4-(7)-...
  /// TODO
}