#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "driver.h"

namespace py = pybind11;

// Helper for zero-copy memory mapping of global float arrays
py::array_t<float> get_mapped_array(float* ptr) {
    // We create a NumPy array that points directly to the C memory address
    return py::array_t<float>(
        { (size_t)MOTOR_COUNT }, // shape
        { sizeof(float) },       // strides (1D float array)
        ptr,                     // data pointer
        py::capsule(ptr, [](void *f) {}) // dummy cleanup (don't free global memory)
    );
}

// Optimized send_command using NumPy buffers
void send_command_fast(py::array_t<float> pos, py::array_t<float> spd) {
    auto r_pos = pos.unchecked<1>();
    auto r_spd = spd.unchecked<1>();

    if (r_pos.shape(0) != MOTOR_COUNT || r_spd.shape(0) != MOTOR_COUNT) {
        throw std::runtime_error("Array size mismatch");
    }

    // Call the original C function using the direct pointers from NumPy
    send_command(r_pos.data(0), r_spd.data(0));
}

PYBIND11_MODULE(motor_driver, m) {
    m.def("initialize", &initialize_driver);
    m.def("resolve", &resolve_command);
    
    // Direct memory views (Zero-copy)
    m.def("get_positions", []() { return get_mapped_array(read_joints_pos()); });
    m.def("get_velocities", []() { return get_mapped_array(read_joints_vel()); });

    // Fast command injection
    m.def("send_command", &send_command_fast);
}
