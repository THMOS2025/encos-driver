/* driver.h
 *  high-level abstract ready for use
 */

#include <stdint.h>

int driver_initialize();
int driver_uninitialize();
int driver_set_id(const uint8_t old_id, const uint8_t new_id);
int driver_set_motor_zero(const uint8_t id);
int driver_push_msg();
int driver_pull_msg();
int driver_set_qpos(const float qpos[]);
int driver_get_qpos_qvel(float qpos[], float qvel[]);
int driver_send_query(const uint8_t code);
