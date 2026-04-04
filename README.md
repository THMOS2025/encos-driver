# Encos Driver

A C/C++ high-level Linux driver for controlling Encos motors over SocketCAN. 
This driver provides an abstracted API to initialize motors, configure them, send target position control commands, and receive motor status (position, velocity, errors).

## Architecture

The project is structured into three main layers:

1. **Hardware Interaction Layer (`socket_can_driver.c/h`)** 
   - Uses the Linux SocketCAN interface (`PF_CAN`, `SOCK_RAW`) to communicate with the hardware CAN interfaces.
   - Operates in a **non-blocking** mode.
   - Handles automatically bringing up the `canX` network interfaces using `struct ifreq` and `ioctl`.

2. **Command Abstraction Layer (`encos_command.c/h`)**
   - Transcodes high-level motor concepts into raw binary CAN messages specific to Encos motors.
   - Scans CAN networks for available motors, keeping track of error states and assigning IDs.
   - Reconstructs position, velocity, and error status from received CAN messages.
   - Computes limits and handles configuration.

3. **High-Level API Layer (`driver.c/h`)**
   - Exposes simplified APIs for applications to control all motors simultaneously.
   - Abstract limits, conversions, and states, providing pure geometric/kinematic variables to the user.

*(Optional: The project also includes Python bindings using pybind11 in `driver_pybind.cpp`.)*
Note: Python bindings are not working yet; only LIBAPI interfaces are currently exposed.

## Dependencies

- **Linux**: This driver requires Linux, specifically the SocketCAN stack (`linux/can.h`). It is **not** supported on Windows.
- **CMake** (v3.12+): Used for building the project.
- **GCC/Clang**: Standard C17 / CXX17 compilation.
- **sudo/iproot privileges**: The script invokes `sudo ip link set ...` internally to configure CAN interfaces, so the executing user needs relevant privileges.
- **Python (Optional)**: If using Python bindings, install `pybind11` and `numpy`.
  ```bash
  pip install pybind11 numpy
  ```

## Building

### 1. Build C Library and Executables
To build the standard driver:

```bash
cd encos-driver
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

This will produce:
- `encos_driver_lib`: A shared library exposing the API.
- `encos_driver`: A reference executable based on `src/main.c`.

### 2. Build Python Bindings
If you want to use the driver from Python:

Note: Python bindings are not working yet; only LIBAPI interfaces are currently exposed.

```bash
cd build
cmake -DBUILD_PYTHON_BINDINGS=ON ..
make
```
This will produce `encos_python.so` (or similar) in the build directory.

## Testing

### C Version
I have provided a simple C test script in `scripts/motor_test.c`. To compile and run it:

```bash
gcc scripts/motor_test.c -Iinclude -Lbuild -lencos_driver_lib -lm -o motor_test
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/build
./motor_test
```

### Python Version
Use the provided `scripts/sysid_chirp.py` or a simple script:

```python
import encos_python
encos_python.initialize()
# ... your logic ...
encos_python.uninitialize()
```

Include `"driver.h"` in your C/C++ application.

### Important Functions

- `int driver_initialize()`
  Brings up the interfaces, polls for motors across all channels, identifies their IDs, and applies default ranges.
  
- `int driver_uninitialize()`
  Gracefully releases socket objects and resources.

- `int driver_set_motor_zero(const uint8_t id)`
  Instructs motor with physical ID `id` to set its current position as the geometric zero.

- `int driver_send_qpos(const float qpos[])`
  Commands an array of target positions for all motors.

- `int driver_pull_msg()`
  Reads incoming status messages from the physical channels (non-blocking) and updates internal motor state buffers.

- `int driver_get_qpos_qvel(float qpos[], float qvel[])`
  Retrieves the internally cached array of positions and velocities. Ensure you invoke `driver_pull_msg()` periodically to keep this buffer updated.

## Example

Reference `src/main.c` for an end-to-end example:

```c
#include <unistd.h>
#include "driver.h"

int main() {
    float qpos[20] = {0};
    float current_qpos[20], current_qvel[20];

    driver_initialize();
    
    // Command motor positions
    qpos[2] = 1.0f;
    driver_send_qpos(qpos);
    usleep(1000);
    
    // Check feedback
    driver_pull_msg();
    driver_get_qpos_qvel(current_qpos, current_qvel);
    
    driver_uninitialize();
    return 0;
}
```

## Internal Limits and Scaling

Positions sent or received are automatically scaled linearly behind the scenes (e.g., bounds such as `-12.5 rad` to `12.5 rad` are mapped to `0-65536` for network transport).
