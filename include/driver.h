/* driver.h
 *  high-level abstract ready for use
 */

#include <stdint.h>

int driver_initialize();
int driver_uninitialize();
int driver_set_motor_zero(const uint8_t id);
int driver_send_qpos(const float qpos[]);
int driver_set_kpkd(const float kp[], const float kd[]);
int driver_pull_msg();
int driver_get_qpos_qvel(float qpos[], float qvel[]);
int driver_get_qpos_qvel_qcur(float qpos[], float qvel[], float qcur[]);

float* read_joints_pos();
float* read_joints_vel();
float* read_joints_cur();
