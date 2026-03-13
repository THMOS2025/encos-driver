/* C executable / Examples of api
 *
 */

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "driver.h"

int main(int argc, char **argv)
{
    float desired_qpos[20] = {1.0, 1.0};
    float current_qpos[20], current_qvel[20];

    driver_initialize();
    for(int j = 0; j < 20; ++j)
        desired_qpos[j] = 1.0;
    usleep(1000000);
    driver_set_motor_zero(2);
    usleep(10000000);
    for(int i = 0; i < 1; ++i) {
        driver_send_qpos(desired_qpos);
        usleep(100000);
        driver_pull_msg();
        driver_get_qpos_qvel(current_qpos, current_qvel);
        printf("cmd: %0.3lf cur: %0.3lf\n", desired_qpos[2], current_qpos[2]);
    }
    usleep(1000000);
    for(int i = 0; i < 10; ++i) {
        driver_pull_msg();
        usleep(100000);
    }
    driver_get_qpos_qvel(current_qpos, current_qvel);
    printf("cmd: %0.3lf cur: %0.3lf\n", desired_qpos[2], current_qpos[2]);
    driver_uninitialize();
}

