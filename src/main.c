/* C executable / Examples of api
 *
 */

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "driver.h"

const uint8_t id = 1;

int main(int argc, char **argv)
{
    float desired_qpos[20] = {1.0, 1.0};
    float current_qpos[20], current_qvel[20];

    driver_initialize();
    // driver_set_motor_zero(1);

    
    printf("ts, cmdpos, curpos, curspd\n");
    for(int i = 0; i < 10000; ++i) {
        float t = i * 0.001;
        desired_qpos[id] = 0.05 * sin(0.5 * t * t);
        driver_set_qpos(desired_qpos);
        driver_push_msg();
        usleep(1000);
        driver_pull_msg();
        driver_get_qpos_qvel(current_qpos, current_qvel);
        if(i > 100)
            printf("%.3lf,%0.3lf,%0.3lf,%0.3lf\n", t, 
                    desired_qpos[id] * 12.5f, 
                    current_qpos[id] * 12.5f,
                    current_qvel[id] * 12.5f);
    }
    
    for(int i = 0; i < 100; ++i) {
        desired_qpos[id] *= 0.9;
        driver_set_qpos(desired_qpos);
        driver_push_msg();
        usleep(10000);
        driver_pull_msg();
    }
    // driver_get_qpos_qvel(current_qpos, current_qvel);
    // printf("%0.3lf,%0.3lf,%0.3lf\n", desired_qpos[id], current_qpos[id], current_qvel[id]); 

    driver_uninitialize();
    return 0;
}

