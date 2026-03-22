#include "constant.h"
#include "driver.h"
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Generate linear chirp signal
float generate_chirp(float t, float f_start, float f_end, float T,
                     float amplitude) {
  float k = (f_end - f_start) / T;
  float phase = 2.0f * M_PI * (f_start * t + 0.5f * k * t * t);
  return amplitude * sinf(phase);
}

// Function to get current time in seconds
double get_time_s() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

int main(int argc, char *argv[]) {
  printf("Starting C Motor SysID Chirp Test\n");

  driver_initialize();
  printf("Driver initialized. MOTOR_COUNT: %d\n", MOTOR_COUNT);

  float qpos_norm[MOTOR_COUNT] = {0};
  float qvel_norm[MOTOR_COUNT] = {0};
  float qtor_norm[MOTOR_COUNT] = {0};
  float target_pos_norm[MOTOR_COUNT] = {0};

  // Test parameters (4310 motor, 5x test)
  int motor_id = 0;                          // Modify as needed
  float f_start = 0.1f;                      // Hz
  float f_end = 40.01f;                      // Hz
  float amp_deg = 1.432178f;                 // 5x test amplitude
  float amplitude = amp_deg * M_PI / 180.0f; // rad
  float T = 20.0f;                           // Duration in seconds
  float fs = 500.0f;                         // Loop frequency in Hz
  double dt = 1.0 / fs;

  printf("Starting Sweep on motor %d: freq %f to %f Hz, Amp %f deg\n", motor_id,
         f_start, f_end, amp_deg);

  sleep(1); // Wait for motor scan

  FILE *f = fopen("sysid_data_c_log.csv", "w");
  if (!f) {
    printf("Failed to open log file!\n");
    return -1;
  }
  fprintf(f, "time,target_pos,actual_pos,actual_vel,actual_tor\n");

  int total_steps = (int)(T * fs);
  double start_time = get_time_s();

  for (int step = 0; step < total_steps; step++) {
    double loop_start = get_time_s();
    float t = step * dt;

    // Calculate physical target
    float target_phys = generate_chirp(t, f_start, f_end, T, amplitude);

    // Scale to normalized ranges defined in constant.c before sending
    // Note: QPOS_RANGE[1] is the positive max value
    target_pos_norm[motor_id] = target_phys / QPOS_RANGE[1][motor_id];

    driver_send_qpos(target_pos_norm);
    driver_pull_msg();
    driver_get_qpos_qvel_qtor(qpos_norm, qvel_norm, qtor_norm);

    // Convert back to physical units for logging based on ranges
    float actual_pos = qpos_norm[motor_id] * QPOS_RANGE[1][motor_id];
    // Velocity range scale isn't explicitly in constant.h arrays.
    // We'll scale it similarly to pos or keep raw.
    // E.g. encos_command.c uses same scale/offset conversion for all three,
    // so we scale by corresponding RANGE if available. Vel might not be
    // strictly bounded in QPOS_RANGE. Assuming velocity and position have
    // similar mapping in the raw msg structure for this driver version.
    // Actually, we'll scale by QPOS_RANGE for pos, and QTOR_RANGE for torque.
    float actual_vel =
        qvel_norm[motor_id] * QPOS_RANGE[1][motor_id]; // Approximation
    float actual_tor = qtor_norm[motor_id] * QTOR_RANGE[1][motor_id];

    // Ensure we don't log more than exact duration to console every second
    if (step % (int)fs == 0) {
      printf("Step %d: Tgt: %.4f, Pos: %.4f, Vel: %.4f, Tor: %.4f\n", step,
             target_phys, actual_pos, actual_vel, actual_tor);
    }

    fprintf(f, "%.4f,%.6f,%.6f,%.6f,%.6f\n", get_time_s() - start_time,
            target_phys, actual_pos, actual_vel, actual_tor);

    // precise wait
    double loop_elapsed = get_time_s() - loop_start;
    if (dt > loop_elapsed) {
      usleep((dt - loop_elapsed) * 1e6);
    }
  }

  fclose(f);
  driver_uninitialize();
  printf("Finished\n");
  return 0;
}
