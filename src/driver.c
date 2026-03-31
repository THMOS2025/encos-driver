/* driver.c
 *  high-level abstract ready for use
 */

#if defined(_WIN32) || defined(_WIN64)
#error("Windows is not supported")
#else
// Linux/Unix logic (GCC/Clang)
#if __GNUC__ >= 4
#define LIB_API __attribute__((visibility("default")))
#else
#define LIB_API
#endif
#endif

#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

#include "driver.h"
#include "constant.h"
#include "encos_command.h"
#include "log.h"

static float qpos[MOTOR_COUNT];
static float qvel[MOTOR_COUNT];
static float qcur[MOTOR_COUNT];

LIB_API float* read_joints_pos() { return qpos; }
LIB_API float* read_joints_vel() { return qvel; }
LIB_API float* read_joints_cur() { return qcur; }

LIB_API int driver_initialize()
{
    log_set_level(LOG_LEVEL);
    log_info("Initializing driver");
    initialize_motors();
    scan_motors(1000000);
    log_info("Sending default settings");
    send_motor_set_pos_range(QPOS_RANGE);
    /* 
       send_motor_set_tor_range(QTOR_RANGE);
       send_motor_set_kp_range(QKP_RANGE);
       send_motor_set_kd_range(QKD_RANGE);
       */
    usleep(100000);
    driver_pull_msg();
}

LIB_API int driver_uninitialize()
{
    return uninitialize_motors();
}

LIB_API int driver_set_motor_zero(const uint8_t id)
{
    return send_motor_set_zero(id);
}

LIB_API int driver_send_qpos(const float qpos[])
{
    return send_motors_pos(qpos);
}

LIB_API int driver_pull_msg()
{
    return pull_motors_msg();
}

LIB_API int driver_get_qpos_qvel(float qpos[], float qvel[])
{
    return get_motors_pos_vel(qpos, qvel);
}

LIB_API int driver_get_qpos_qvel_qcur(float qpos[], float qvel[], float qcur[])
{
    return get_motors_pos_vel_cur(qpos, qvel, qcur);
}

LIB_API int driver_send_query(const uint8_t code)
{
    return send_motors_query(code);
}
