// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

#pragma once

// raisim include
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

#include "../../Yaml.hpp"
#include "../../BasicEigenTypes.hpp"
#include "../../Reward.hpp"
#include PLAYER1_HEADER_FILE_TO_INCLUDE
#include PLAYER2_HEADER_FILE_TO_INCLUDE

namespace raisim {

class ENVIRONMENT_for_test {

 public:

  explicit ENVIRONMENT_for_test(const std::string &resourceDir, const Yaml::Node &cfg, bool visualizable) :
      visualizable_(visualizable) {
    /// add objects
    auto* robot = world_.addArticulatedSystem(resourceDir + "/anymal/urdf/anymal_red.urdf");
    robot->setName(PLAYER1_NAME);
    std::cout<<"player 1 name: "<<PLAYER1_NAME<<std::endl;
    robot->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);


    ///////////////////////////////////////////////////////////////////////////////////////////////
    /// if you want make opponent robot, use like below code (but in Environment.hpp, there exist only PLAYER_NAME in definition. So use any name for opponent robot name setting.
    auto* dummy_robot = world_.addArticulatedSystem(resourceDir + "/anymal/urdf/anymal_blue.urdf");
    dummy_robot->setName(PLAYER2_NAME);
    std::cout<<"player 2 name: "<<PLAYER2_NAME<<std::endl;
    dummy_robot->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);


    controller_.setName(PLAYER1_NAME);
    controller_.setOpponentName(PLAYER2_NAME);
    dummyController_.setName(PLAYER2_NAME);
    dummyController_.setOpponentName(PLAYER1_NAME);
    ///////////////////////////////////////////////////////////////////////////////////////////////

    auto* ground = world_.addGround();
    ground->setName("ground");

    controller_.create(&world_);
    dummyController_.create(&world_);
    READ_YAML(double, simulation_dt_, cfg["simulation_dt"])
    READ_YAML(double, control_dt_, cfg["control_dt"])

    /// visualize if it is the first environment
    if (visualizable_) {
      server_ = std::make_unique<raisim::RaisimServer>(&world_);
      server_->launchServer();
      auto cage = server_->addVisualCylinder("cage", 3.0, 0.1);
      cage->setPosition(0,0,0);
    }
  }

  ~ENVIRONMENT_for_test() {
    if(server_) server_->killServer();
  }

  void init() {}

  void reset() {
    double theta = uniDist_(gen_) * 2 * M_PI;
//    theta = 0;
    controller_.reset(&world_, theta);
    dummyController_.reset(&world_, theta);
    timer_ = 0;
  }

  void step(const Eigen::Ref<EigenVec> &action) {
    timer_ += 1;
    controller_.advance(&world_, action);
    dummyController_.advance(&world_, action);
    for (int i = 0; i < int(control_dt_ / simulation_dt_ + 1e-10); i++) {
      if (server_) server_->lockVisualizationServerMutex();
      world_.integrate();
      if (server_) server_->unlockVisualizationServerMutex();
    }
    controller_.updateObservation(&world_);
    dummyController_.updateObservation(&world_);
  }

  void observe(Eigen::Ref<EigenVec> ob) {
    controller_.updateObservation(&world_);
    dummyController_.updateObservation(&world_);
    ob = controller_.getObservation().cast<float>();
  }

  bool player1_die() {
    auto anymal = reinterpret_cast<raisim::ArticulatedSystem *>(world_.getObject(PLAYER1_NAME));
    /// base contact with ground
    for(auto& contact: anymal->getContacts()) {
      if(contact.getPairObjectIndex() == world_.getObject("ground")->getIndexInWorld() &&
          contact.getlocalBodyIndex() == anymal->getBodyIdx("base")) {
        return true;
      }
    }
    /// get out of the cage
    int gcDim = anymal->getGeneralizedCoordinateDim();
    Eigen::VectorXd gc;
    gc.setZero(gcDim);
    gc = anymal->getGeneralizedCoordinate().e();
    if (gc.head(2).norm() > 3) {
      return true;
    }
    return false;
  }

  bool player2_die() {
    auto anymal = reinterpret_cast<raisim::ArticulatedSystem *>(world_.getObject(PLAYER2_NAME));
    /// base contact with ground
    for(auto& contact: anymal->getContacts()) {
      if(contact.getPairObjectIndex() == world_.getObject("ground")->getIndexInWorld() &&
          contact.getlocalBodyIndex() == anymal->getBodyIdx("base")) {
        return true;
      }
    }
    /// get out of the cage
    int gcDim = anymal->getGeneralizedCoordinateDim();
    Eigen::VectorXd gc;
    gc.setZero(gcDim);
    gc = anymal->getGeneralizedCoordinate().e();
    if (gc.head(2).norm() > 3) {
      return true;
    }
    return false;
  }

  bool isTerminalState() {
    if (player1_die() && player2_die()) {
      draw += 1;
      terminal += 1;
      std::cout<<"draw"<<std::endl;
      return true;
    }

    if (timer_ > 10 * 100) {
      draw += 1;
      terminal += 1;
      std::cout<<"draw"<<std::endl;
      return true;
    }


    if (!player1_die() && player2_die()) {
      player1_win += 1;
      terminal += 1;
      std::cout<<"player1 win"<<std::endl;
      return true;
    }

    if (player1_die() && !player2_die()) {
      player2_win += 1;
      terminal += 1;
      std::cout<<"player2 win"<<std::endl;
      return true;
    }

    if (terminal == 10) {
      std::cout<<"finish!"<<"\t"<<"player1 win: "<<player1_win<<"\t"<<"player2 win: "<<player2_win<<"\t"<<"draw: "<<draw<<std::endl;
      std::cout<<"player1 get: "<<3 * player1_win + draw<<"\t"<<"player2 get: "<<3 * player2_win + draw<<std::endl;
      exit(0);
    }
    return false;
  }

  void curriculumUpdate() {};

  void close() { if (server_) server_->killServer(); };

  void setSeed(int seed) {};

  void setSimulationTimeStep(double dt) {
    simulation_dt_ = dt;
    world_.setTimeStep(dt);
  }
  void setControlTimeStep(double dt) { control_dt_ = dt; }

  int getObDim() { return controller_.getObDim(); }

  int getActionDim() { return controller_.getActionDim(); }

  double getControlTimeStep() { return control_dt_; }

  double getSimulationTimeStep() { return simulation_dt_; }

  raisim::World *getWorld() { return &world_; }

  void turnOffVisualization() { server_->hibernate(); }

  void turnOnVisualization() { server_->wakeup(); }

  void startRecordingVideo(const std::string &videoName) { server_->startRecordingVideo(videoName); }

  void stopRecordingVideo() { server_->stopRecordingVideo(); }

 private:
  int timer_ = 0;
  int player1_win = 0, player2_win = 0, draw = 0, terminal = 0;
  bool visualizable_ = false;
  PLAYER1_CONTROLLER controller_;
  PLAYER2_CONTROLLER dummyController_;
  raisim::World world_;
  double simulation_dt_ = 0.001;
  double control_dt_ = 0.01;
  std::unique_ptr<raisim::RaisimServer> server_;
  std::uniform_real_distribution<double> uniDist_;
  std::mt19937 gen_;
};
}

