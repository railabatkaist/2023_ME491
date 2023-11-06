//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "Environment_for_test.hpp"
#include "VectorizedEnvironment_for_test.hpp"

namespace py = pybind11;
using namespace raisim;

PYBIND11_MODULE(RAISIMGYM_TORCH_ENV_NAME, m) {
  py::class_<VectorizedEnvironment_for_test<ENVIRONMENT_for_test>>(m, "RaisimGymEnv")
    .def(py::init<std::string, std::string>())
    .def("init", &VectorizedEnvironment_for_test<ENVIRONMENT_for_test>::init)
    .def("reset", &VectorizedEnvironment_for_test<ENVIRONMENT_for_test>::reset)
    .def("observe", &VectorizedEnvironment_for_test<ENVIRONMENT_for_test>::observe)
    .def("step", &VectorizedEnvironment_for_test<ENVIRONMENT_for_test>::step)
    .def("setSeed", &VectorizedEnvironment_for_test<ENVIRONMENT_for_test>::setSeed)
    .def("close", &VectorizedEnvironment_for_test<ENVIRONMENT_for_test>::close)
    .def("isTerminalState", &VectorizedEnvironment_for_test<ENVIRONMENT_for_test>::isTerminalState)
    .def("setSimulationTimeStep", &VectorizedEnvironment_for_test<ENVIRONMENT_for_test>::setSimulationTimeStep)
    .def("setControlTimeStep", &VectorizedEnvironment_for_test<ENVIRONMENT_for_test>::setControlTimeStep)
    .def("getObDim", &VectorizedEnvironment_for_test<ENVIRONMENT_for_test>::getObDim)
    .def("getActionDim", &VectorizedEnvironment_for_test<ENVIRONMENT_for_test>::getActionDim)
    .def("getNumOfEnvs", &VectorizedEnvironment_for_test<ENVIRONMENT_for_test>::getNumOfEnvs)
    .def("turnOnVisualization", &VectorizedEnvironment_for_test<ENVIRONMENT_for_test>::turnOnVisualization)
    .def("turnOffVisualization", &VectorizedEnvironment_for_test<ENVIRONMENT_for_test>::turnOffVisualization)
    .def("stopRecordingVideo", &VectorizedEnvironment_for_test<ENVIRONMENT_for_test>::stopRecordingVideo)
    .def("startRecordingVideo", &VectorizedEnvironment_for_test<ENVIRONMENT_for_test>::startRecordingVideo)
    .def("curriculumUpdate", &VectorizedEnvironment_for_test<ENVIRONMENT_for_test>::curriculumUpdate);
}
