#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include "driver.h"
#include "constant.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_TESTS 32
#define MAX_LINE  512

typedef struct {
    char motor_model[16];
    int  motor_id;
    float J;
    float zeta;
    float omega_n;
    float kp;
    float kd;
    float f_start;
    float f_end;
    float amp_deg;
    float duration;
    float fs;
} test_config_t;

// Generate linear chirp signal
float generate_chirp(float t, float f_start, float f_end, float T, float amplitude) {
    float k = (f_end - f_start) / T;
    float phase = 2.0f * M_PI * (f_start * t + 0.5f * k * t * t);
    return amplitude * sinf(phase);
}

// Get current time in seconds
double get_time_s() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// Parse CSV config file, returns number of tests loaded
int load_config(const char* path, test_config_t tests[], int max_tests) {
    FILE *f = fopen(path, "r");
    if (!f) {
        printf("ERROR: Cannot open config file: %s\n", path);
        return -1;
    }
    char line[MAX_LINE];
    int count = 0;
    while (fgets(line, sizeof(line), f) && count < max_tests) {
        // Skip comments and empty lines
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
        test_config_t *t = &tests[count];
        int ret = sscanf(line, "%15[^,],%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
            t->motor_model, &t->motor_id,
            &t->J, &t->zeta, &t->omega_n,
            &t->kp, &t->kd,
            &t->f_start, &t->f_end, &t->amp_deg,
            &t->duration, &t->fs);
        if (ret == 12) {
            ++count;
        } else {
            printf("WARN: Skipping malformed line: %s", line);
        }
    }
    fclose(f);
    return count;
}

// Run a single chirp test
int run_chirp_test(const test_config_t *cfg, int test_index) {
    int mid = cfg->motor_id;
    float amplitude = cfg->amp_deg * M_PI / 180.0f; // deg -> rad
    double dt = 1.0 / cfg->fs;
    int total_steps = (int)(cfg->duration * cfg->fs);

    printf("\n========================================\n");
    printf("Test %d: %s motor_id=%d zeta=%.3f omega_n=%.1f\n",
           test_index, cfg->motor_model, mid, cfg->zeta, cfg->omega_n);
    printf("  Kp=%.4f  Kd=%.4f\n", cfg->kp, cfg->kd);
    printf("  Sweep: %.2f -> %.2f Hz, Amp=%.4f deg, T=%.1f s, fs=%.0f Hz\n",
           cfg->f_start, cfg->f_end, cfg->amp_deg, cfg->duration, cfg->fs);
    printf("========================================\n");

    // Set Kp/Kd for all motors (only motor_id matters for the test)
    float kp_arr[MOTOR_COUNT] = {0};
    float kd_arr[MOTOR_COUNT] = {0};
    kp_arr[mid] = cfg->kp;
    kd_arr[mid] = cfg->kd;
    driver_set_kpkd(kp_arr, kd_arr);

    // Open log file
    char filename[128];
    snprintf(filename, sizeof(filename),
        "sysid_%s_z%.1f_kp%.1f_kd%.2f.csv",
        cfg->motor_model, cfg->zeta, cfg->kp, cfg->kd);
    FILE *logf = fopen(filename, "w");
    if (!logf) {
        printf("ERROR: Cannot open log file: %s\n", filename);
        return -1;
    }
    fprintf(logf, "time,target_pos,actual_pos,actual_vel,actual_tor\n");

    float target_pos_norm[MOTOR_COUNT] = {0};
    float qpos_norm[MOTOR_COUNT] = {0};
    float qvel_norm[MOTOR_COUNT] = {0};
    float qtor_norm[MOTOR_COUNT] = {0};

    double start_time = get_time_s();

    for (int step = 0; step < total_steps; step++) {
        double loop_start = get_time_s();
        float t = step * dt;

        // Generate chirp target (physical rad)
        float target_phys = generate_chirp(t, cfg->f_start, cfg->f_end,
                                           cfg->duration, amplitude);

        // Normalize to [-1, 1] using QPOS_RANGE before sending
        target_pos_norm[mid] = target_phys / QPOS_RANGE[1][mid];

        driver_send_qpos(target_pos_norm);
        driver_pull_msg();
        driver_get_qpos_qvel_qtor(qpos_norm, qvel_norm, qtor_norm);

        // Convert normalized readings back to physical values
        float actual_pos = qpos_norm[mid] * QPOS_RANGE[1][mid];
        float actual_vel = qvel_norm[mid] * QPOS_RANGE[1][mid];
        float actual_tor = qtor_norm[mid] * QTOR_RANGE[1][mid];

        // Console output every second
        if (step % (int)cfg->fs == 0) {
            printf("  [%.1fs] Tgt=%.4f Pos=%.4f Vel=%.4f Tor=%.4f\n",
                   t, target_phys, actual_pos, actual_vel, actual_tor);
        }

        fprintf(logf, "%.4f,%.6f,%.6f,%.6f,%.6f\n",
                get_time_s() - start_time,
                target_phys, actual_pos, actual_vel, actual_tor);

        // Precise timing
        double loop_elapsed = get_time_s() - loop_start;
        if (dt > loop_elapsed) {
            usleep((useconds_t)((dt - loop_elapsed) * 1e6));
        }
    }

    fclose(logf);
    printf("  -> Saved to %s\n", filename);

    // Rest between tests: return to zero and wait
    memset(target_pos_norm, 0, sizeof(target_pos_norm));
    driver_send_qpos(target_pos_norm);
    sleep(2);

    return 0;
}

int main(int argc, char* argv[]) {
    const char *config_path = "config/sysid_tests.csv";
    if (argc > 1) config_path = argv[1];

    printf("=== SysID Chirp Test Suite ===\n");
    printf("Config: %s\n", config_path);

    // Load test configs
    test_config_t tests[MAX_TESTS];
    int n_tests = load_config(config_path, tests, MAX_TESTS);
    if (n_tests <= 0) {
        printf("No valid tests found in config. Exiting.\n");
        return -1;
    }
    printf("Loaded %d test(s)\n", n_tests);

    // Initialize driver
    driver_initialize();
    printf("Driver initialized. MOTOR_COUNT: %d\n", MOTOR_COUNT);
    sleep(1);

    // Run each test
    for (int i = 0; i < n_tests; i++) {
        run_chirp_test(&tests[i], i + 1);
    }

    driver_uninitialize();
    printf("\n=== All tests completed ===\n");
    return 0;
}
