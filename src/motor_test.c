#include "constant.h"
#include "driver.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_TESTS 32
#define MAX_LINE 512

typedef struct {
  char motor_model[16];
  int motor_id;
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
float generate_chirp(float t, float f_start, float f_end, float T,
                     float amplitude) {
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
int load_config(const char *path, test_config_t tests[], int max_tests) {
  FILE *f = fopen(path, "r");
  if (!f) {
    printf("ERROR: Cannot open config file: %s\n", path);
    return -1;
  }
  char line[MAX_LINE];
  int count = 0;
  while (fgets(line, sizeof(line), f) && count < max_tests) {
    // Skip comments and empty lines
    if (line[0] == '#' || line[0] == '\n' || line[0] == '\r')
      continue;
    test_config_t *t = &tests[count];
    int ret =
        sscanf(line, "%15[^,],%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", t->motor_model,
               &t->motor_id, &t->J, &t->zeta, &t->omega_n, &t->kp, &t->kd,
               &t->f_start, &t->f_end, &t->amp_deg, &t->duration, &t->fs);
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
  printf("Test %d: %s motor_id=%d zeta=%.3f omega_n=%.1f\n", test_index,
         cfg->motor_model, mid, cfg->zeta, cfg->omega_n);
  printf("  Kp=%.4f  Kd=%.4f\n", cfg->kp, cfg->kd);
  printf("  Sweep: %.2f -> %.2f Hz, Amp=%.4f deg, T=%.1f s, fs=%.0f Hz\n",
         cfg->f_start, cfg->f_end, cfg->amp_deg, cfg->duration, cfg->fs);
  printf("========================================\n");

  // Set Kp/Kd for all motors
  float kp_arr[MOTOR_COUNT] = {0};
  float kd_arr[MOTOR_COUNT] = {0};
  kp_arr[mid] = cfg->kp;
  kd_arr[mid] = cfg->kd;
  driver_set_kpkd(kp_arr, kd_arr);
  printf("[DEBUG] driver_set_kpkd called\n");

  // ==========================================
  // 关键修复区 1: 电机置零与使能序列 (对齐成功案例)
  // ==========================================
  printf("[DEBUG] Zeroing/Enabling motor %d...\n", mid);
  driver_set_motor_zero(mid);
  usleep(1000000); // 必须等待足够长的时间让电机完成置零动作 (1秒)

  // 关键修复区 2: 循环前拉取初始状态，打通通讯链路
  driver_pull_msg();
  // ==========================================

  // Open log file
  char filename[128];
  snprintf(filename, sizeof(filename), "sysid_%s_z%.1f_kp%.1f_kd%.2f.csv",
           cfg->motor_model, cfg->zeta, cfg->kp, cfg->kd);
  FILE *logf = fopen(filename, "w");
  if (!logf) {
    printf("ERROR: Cannot open log file: %s\n", filename);
    return -1;
  }
  fprintf(logf, "time,target_pos,actual_pos,actual_vel,actual_cur\n");

  float target_pos[MOTOR_COUNT] = {0};
  float qpos[MOTOR_COUNT] = {0};
  float qvel[MOTOR_COUNT] = {0};
  float qcur[MOTOR_COUNT] = {0};

  double start_time = get_time_s();

  for (int step = 0; step < total_steps; step++) {
    double loop_start = get_time_s();
    float t = step * dt;

    // Generate chirp target (physical rad)
    float target_phys =
        generate_chirp(t, cfg->f_start, cfg->f_end, cfg->duration, amplitude);

    target_pos[mid] = target_phys;

    // 1. 发送期望位置
    driver_send_qpos(target_pos);

    // ==========================================
    // 关键修复区 3: 时序调整 (Send -> Sleep -> Pull)
    // 在发送指令后，优先等待dt周期耗尽，再拉取状态
    // 这样电机才有时间响应指令并返回最新的物理状态
    // ==========================================
    double loop_elapsed = get_time_s() - loop_start;
    if (dt > loop_elapsed) {
      usleep((useconds_t)((dt - loop_elapsed) * 1e6));
    }

    // 2. 延迟结束后，拉取并获取最新状态
    driver_pull_msg();
    driver_get_qpos_qvel_qcur(qpos, qvel, qcur);

    float actual_pos = qpos[mid];
    float actual_vel = qvel[mid];
    float actual_cur = qcur[mid];

    // Console output every second
    if (step % (int)cfg->fs == 0) {
      printf("  [%.1fs] Tgt=%.4f Pos=%.4f Vel=%.4f Cur=%.4f\n", t, target_phys,
             actual_pos, actual_vel, actual_cur);
    }

    // 记录日志时使用循环开头计算的时间点，保证时间戳与生成的波形一致
    fprintf(logf, "%.4f,%.6f,%.6f,%.6f,%.6f\n",
            t, // 使用理想时间轴更利于SysID数据分析
            target_phys, actual_pos, actual_vel, actual_cur);
  }

  fclose(logf);
  printf("  -> Saved to %s\n", filename);

  // Rest between tests: return to zero and wait
  memset(target_pos, 0, sizeof(target_pos));
  driver_send_qpos(target_pos);

  // 给电机归零留出充足时间并再次拉取消息保持链路活跃
  usleep(2000000); // sleep(2)
  driver_pull_msg();

  return 0;
}

int main(int argc, char *argv[]) {
  const char *config_path = "config/sysid_tests.csv";
  if (argc > 1)
    config_path = argv[1];

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
  usleep(1000000); // 替换为 usleep 保持代码风格一致，让总线有时间准备

  // Run each test
  for (int i = 0; i < n_tests; i++) {
    run_chirp_test(&tests[i], i + 1);
  }

  driver_uninitialize();
  printf("\n=== All tests completed ===\n");
  return 0;
}