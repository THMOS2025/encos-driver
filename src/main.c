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
    // driver_set_motor_zero(2);
    usleep(1000000);
    driver_pull_msg();

    driver_send_query(23);
    usleep(1000000);
    driver_pull_msg();

    driver_send_query(1);
    usleep(1000000);
    driver_pull_msg();

    return 0;
    printf("ts, cmdpos, curpos, curspd\n");
    for(int i = 0; i < 10000; ++i) {
        float t = i * 0.001;
        for(int j = 0; j < 20; ++j)
            desired_qpos[j] = sin(2.0 * t * t);
        driver_send_qpos(desired_qpos);
        usleep(1000);
        driver_pull_msg();
        driver_get_qpos_qvel(current_qpos, current_qvel);
        printf("%.3lf,%0.3lf,%0.3lf,%0.3lf\n", t, 
                desired_qpos[2], 
                current_qpos[2],
                current_qvel[2]);
    }
    driver_pull_msg();
    driver_get_qpos_qvel(current_qpos, current_qvel);
    printf("cmd: %0.3lf cur: %0.3lf\n", desired_qpos[3], current_qpos[3]);
    driver_uninitialize();
    return 0;
}

