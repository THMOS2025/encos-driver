/* encos_command.h
 *      wrapping binaray encos commands into apis
 */

#ifndef _H_ENCOS_COMMAND
#define _H_ENCOS_COMMAND

#include <stdint.h>
#include "constant.h"

/* Meta */
int initialize_motors();
int uninitialize_motors();
int send_motor_set_zero(const uint8_t id);
int scan_motors(const uint32_t timeout_us);

/* Core */
int send_motors_pos(const float qpos[MOTOR_COUNT]);
int set_motors_kpkd(const float kp[MOTOR_COUNT], const float kd[MOTOR_COUNT]);
int pull_motors_msg();
int get_motors_pos_vel(float qpos[MOTOR_COUNT], float qvel[MOTOR_COUNT]);
int get_motors_pos_vel_tor(float qpos[MOTOR_COUNT], float qvel[MOTOR_COUNT], float qtor[MOTOR_COUNT]);

/* Config */
int send_motor_set_pos_range(const float qpos_range[2][MOTOR_COUNT]);
int send_motor_set_vel_range(const float qpos_range[2][MOTOR_COUNT]);
int send_motor_set_tor_range(const float qtor_range[2][MOTOR_COUNT]);
int send_motor_set_cur_range(const float qcur_range[2][MOTOR_COUNT]);
int send_motor_set_kp_range(const float qkp_range[2][MOTOR_COUNT]);
int send_motor_set_kd_range(const float qkd_range[2][MOTOR_COUNT]);

#endif
