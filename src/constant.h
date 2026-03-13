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

#endif 
