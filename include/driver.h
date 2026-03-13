/* driver.h
 *  high-level abstract ready for use
 */


int initialize_driver();
int uninitialize_driver();
int send_command(const float target_pos[], const float target_spd[]);
int resolve_command();
const float* read_joints_pos();
const float* read_joints_vel();
