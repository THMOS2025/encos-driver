/* driver.h
 *  high-level abstract ready for use
 */

#include <stdint.h>

int driver_initialize();
int driver_uninitialize();
int driver_set_motor_zero(const uint8_t id);
int driver_send_qpos(const float qpos[]);
int driver_pull_msg();
int driver_get_qpos_qvel(float qpos[], float qvel[]);
