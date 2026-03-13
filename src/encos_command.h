/* encos_command.h
 *      wrapping binaray encos commands into apis
 */

#ifndef _H_ENCOS_COMMAND
#define _H_ENCOS_COMMAND

#include <stdint.h>

int initialize_motors();
int uninitialize_motors();
int send_motor_set_zero(const uint8_t id);
int scan_motors(const uint32_t timeout_us);
int send_motors_pos(const float qpos[]);
int pull_motors_msg();
int get_motors_pos_vel(float qpos[], float qvel[]);

#endif
