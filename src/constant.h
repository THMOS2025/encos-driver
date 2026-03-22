#ifndef _H_DEFINE
#define _H_DEFINE

#include <stdint.h>

#define MOTOR_COUNT                         (20)
#define CHANNEL_COUNT                       (4)

extern const int                            COMMAND_SUCCESS;
extern const int                            COMMAND_FAILED;
extern const int                            NOT_RELATED_MSG;
extern const float                          PAI;
extern const uint32_t                       INITIAL_SCAN_MOTOR_TIME;
extern const uint8_t                        MAX_SEND_FAILED;
extern const int                            LOG_LEVEL;

extern const float                          QPOS_RANGE[2][MOTOR_COUNT];
extern const float                          QTOR_RANGE[2][MOTOR_COUNT];
extern const float                          QKP_RANGE[2][MOTOR_COUNT];
extern const float                          QKD_RANGE[2][MOTOR_COUNT];

extern const char* config_code_to_name[16];
extern const uint8_t CONFIG_ACCLERATION;
extern const uint8_t CONFIG_KT;
extern const uint8_t CONFIG_KP_RANGE;
extern const uint8_t CONFIG_KD_RANGE;
extern const uint8_t CONFIG_POS_RANGE;
extern const uint8_t CONFIG_VEL_RANGE;
extern const uint8_t CONFIG_TOR_RANGE;
extern const uint8_t CONFIG_CUR_RANGE;

#endif 
