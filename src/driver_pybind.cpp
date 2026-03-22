#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>
#include <stdexcept>

extern "C" {
    #include "driver.h"
    #include "constant.h" // To get MOTOR_COUNT
}

namespace py = pybind11;

PYBIND11_MODULE(encos_python, m) {
    m.doc() = "Python bindings for Encos Motor Driver";

    m.def("initialize", &driver_initialize, "Initialize the motor driver");
    
    m.def("uninitialize", &driver_uninitialize, "Uninitialize the motor driver");

    m.def("set_motor_zero", &driver_set_motor_zero, py::arg("id"), 
          "Set the current position of a specific motor as zero");

    m.def("pull_msg", &driver_pull_msg, "Pull messages from the CAN bus");

    // Wrapper for sending positions: accepts a list/vector of floats
    m.def("send_qpos", [](const std::vector<float>& pos) {
        if (pos.size() != MOTOR_COUNT) {
            throw std::runtime_error("Input size must match MOTOR_COUNT (" + std::to_string(MOTOR_COUNT) + ")");
        }
        return driver_send_qpos(pos.data());
    }, "Send target positions to all motors");

    // Wrapper for getting positions/velocities: returns a tuple of two lists
    m.def("get_qpos_qvel", []() {
        std::vector<float> qpos(MOTOR_COUNT);
        std::vector<float> qvel(MOTOR_COUNT);
        
        int result = driver_get_qpos_qvel(qpos.data(), qvel.data());
        
        if (result < 0) {
            throw std::runtime_error("Failed to get motor positions and velocities");
        }

        return std::make_tuple(qpos, qvel);
    }, "Get current positions and velocities of all motors");

    // Export the MOTOR_COUNT constant so Python knows the limit
    m.attr("MOTOR_COUNT") = MOTOR_COUNT;
}
