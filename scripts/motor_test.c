#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "driver.h"
#include "constant.h"

int main() {
    printf("Starting C Motor Test\n");

    if (driver_initialize() != 0) {
        printf("Failed to initialize driver\n");
        return -1;
    }

    printf("Driver initialized. MOTOR_COUNT: %d\n", MOTOR_COUNT);

    float qpos[MOTOR_COUNT] = {0};
    float qvel[MOTOR_COUNT] = {0};
    float target_pos[MOTOR_COUNT] = {0};

    // Test sequence
    for (int step = 0; step < 100; step++) {
        // Simple sine wave for motor 0
        target_pos[0] = 0.5f * sinf(step * 0.1f);
        
        driver_send_qpos(target_pos);
        driver_pull_msg();
        
        driver_get_qpos_qvel(qpos, qvel);
        
        printf("Step %d: Motor 0 Pos: %.3f, Vel: %.3f\n", step, qpos[0], qvel[0]);
        usleep(10000); // 10ms
    }

    driver_uninitialize();
    printf("Finished\n");
    return 0;
}
