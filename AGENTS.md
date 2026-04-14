# Encos Driver — Agent Guide

> This file is intended for AI coding agents. The reader is expected to know nothing about the project.

## 1. Project Overview

**Encos Driver** is a C/C++ high-level Linux driver for controlling Encos motors over SocketCAN. It abstracts raw CAN communication into a simple C API (and experimental Python bindings) for initializing motors, sending position commands, and reading back position/velocity/status.

- **Language**: C17, C++17 (bindings only)
- **Build System**: CMake 3.12+
- **Platform**: Linux only (requires SocketCAN stack). Windows is explicitly unsupported.
- **Max Scale**: 20 motors (`MOTOR_COUNT`) across 4 CAN channels (`CHANNEL_COUNT`).

## 2. Repository Layout

```
├── include/
│   └── driver.h               # Public high-level API header
├── src/
│   ├── driver.c / driver.h    # High-level API implementation
│   ├── encos_command.c / .h   # CAN command encoding / decoding / scanning
│   ├── socket_can_driver.c / .h  # Linux SocketCAN hardware layer
│   ├── constant.c / .h        # Compile-time constants and config arrays
│   ├── main.c                 # Reference C executable
│   ├── motor_test.c           # SysID chirp test suite (C version)
│   └── driver_pybind.cpp      # pybind11 Python bindings
├── deps/
│   └── log.c/                 # Third-party MIT logging library (rxi/log.c)
├── scripts/
│   └── sysid_chirp.py         # SysID chirp test suite (Python version)
├── config/
│   └── sysid_tests.csv        # CSV config for motor_test.c chirp tests
├── CMakeLists.txt             # Primary build configuration
├── pyproject.toml             # Python packaging (scikit-build-core)
└── setup.py                   # Alternative setuptools build with CMakeExtension
```

## 3. Architecture

The code is organized in three layers:

1. **Hardware Interaction Layer** (`socket_can_driver.c/h`)
   - Opens `PF_CAN` / `SOCK_RAW` sockets.
   - Brings up `can0`–`can3` interfaces via `sudo ip link set ...` internally.
   - Operates in **non-blocking** mode.

2. **Command Abstraction Layer** (`encos_command.c/h`)
   - Builds raw 64-bit CAN payloads for the Encos motor protocol.
   - Scans the bus to discover motor IDs and map them to channels.
   - Parses status frames (position, velocity, current, error codes).

3. **High-Level API Layer** (`driver.c/h`)
   - Exposes a small set of functions like `driver_initialize()`, `driver_set_qpos()`, `driver_pull_msg()`, `driver_get_qpos_qvel()`.
   - Handles scaling between normalized float values ([-1, 1] internally, mapped to 0–65535 on the wire) and physical radians (default ±12.5 rad).

## 4. Build Commands

### C Library & Executables
```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```
Outputs:
- `libencos_driver_lib.so` — shared library
- `encos_driver` — reference executable (from `src/main.c`)
- `main` — duplicate executable (also from `src/main.c`)

### Python Bindings (Experimental)
> **Caveat**: The README explicitly states *"Python bindings are not working yet; only LIBAPI interfaces are currently exposed."*

```bash
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDINGS=ON ..
make
```

Or install via Python packaging:
```bash
pip install .
```
This uses `pyproject.toml` (scikit-build-core) which automatically passes `-DBUILD_PYTHON_BINDINGS=ON` to CMake.

## 5. Testing & Validation

There is **no formal unit-test framework** in this repository. Validation is done through hardware-in-the-loop system-identification (SysID) chirp tests.

### C Test (`src/motor_test.c`)
- **Not built by default** — the target is commented out in `CMakeLists.txt`.
- The file contains chirp-test logic that reads `config/sysid_tests.csv` and sweeps motor positions while logging feedback to CSV.
- **Important**: It calls functions (`driver_set_kpkd`, `driver_get_qpos_qvel_qcur`, `driver_send_qpos`) that do **not** exist in the public `driver.h`. If you need to build it, you will likely have to adapt it to the actual API (`driver_set_qpos`, `driver_get_qpos_qvel`, etc.) or extend `driver.h`/`driver.c` accordingly.

### Python Test (`scripts/sysid_chirp.py`)
- Depends on the `encos_python` module.
- Assumes a 20-element command array (hard-coded `np.zeros(20)`).

### Manual C Test (from README)
```bash
gcc scripts/motor_test.c -Iinclude -Lbuild -lencos_driver_lib -lm -o motor_test
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/build
./motor_test
```
*(Note: the README references `scripts/motor_test.c`, but the actual file is `src/motor_test.c`.)*

## 6. Code Style Guidelines

- **C Standard**: C17. **C++ Standard**: C++17 (bindings only).
- Use `/* block comments */` for file-level / section headers.
- Shared-library exports use the `LIB_API` macro (`__attribute__((visibility("default")))` on GCC/Clang).
- Logging: use the `log_trace`, `log_debug`, `log_info`, `log_warn`, `log_error` macros from `deps/log.c/src/log.h` rather than raw `printf`.
- Keep hardware-specific code inside `socket_can_driver.c/h`.
- Keep protocol-specific bit-packing inside `encos_command.c/h`.
- Keep public API surface minimal in `driver.h`.

## 7. Security Considerations

- **Privilege escalation via `system()`**: `socket_can_driver.c` constructs and executes `sudo ip link set can<N> up type can bitrate 1000000 ...` using `sprintf` + `system()`. Ensure `channel` inputs are bounded (they are `uint8_t` and checked against `CHANNEL_COUNT`).
- **Linux-only kernel dependencies**: The code directly includes `<linux/can.h>`, `<linux/can/raw.h>`, `<linux/netlink.h>`, etc. It will not compile on non-Linux systems.
- **No input sanitization on CAN payloads** beyond basic range checks; malformed hardware responses are logged and ignored.

## 8. Known Issues & Agent Caveats

- **Python bindings are incomplete/non-functional** per the README. Do not assume pybind11 targets work out of the box.
- **`src/motor_test.c` is out of sync with `driver.h`**. It references a richer API (`driver_send_qpos`, `driver_get_qpos_qvel_qcur`, `driver_set_kpkd`) than what currently exists. Treat this file as a reference for test logic, not as compilable code without modifications.
- **Several functions have missing return statements** (e.g., `driver_initialize`, `push_motors_msg`, `pull_motors_msg`, `send_motor_set_id`, `send_motors_set_range`, `parse_motor_query`). This triggers compiler warnings and should be fixed if you touch those functions.
- **No CI / automated testing** — any changes must be validated manually on a Linux machine with actual SocketCAN hardware.
