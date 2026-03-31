#include <stdint.h>

#include "constant.h"
#include "log.h"

const float PAI = 3.1415926535897;

/* return definitions */
/* TODO: isolated each error */
const int COMMAND_SUCCESS = 0;
const int COMMAND_FAILED = -1;
const int NOT_RELATED_MSG = 2;

/* configurations */
const uint8_t MAX_SEND_FAILED =
    100; /* after 100 send failed, consider motor loss */
const uint32_t INITIAL_SCAN_MOTOR_TIME = 4;

const int LOG_LEVEL = LOG_TRACE;

/* Limit qpos: physical range in rad. Sent as int16 * scaler(100) to motor. */
const float QPOS_RANGE[2][MOTOR_COUNT] = {
    {-12.5f, -12.5f, -12.5f, -12.5f, -12.5f, -12.5f, -12.5f,
        -12.5f, -12.5f, -12.5f, -12.5f, -12.5f, -12.5f, -12.5f,
        -12.5f, -12.5f, -12.5f, -12.5f, -12.5f, -12.5f},
    {12.5f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f,
        12.5f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f, 12.5f}};

/* forward torque
 *      Currently only use the middle
 */
const float QTOR_RANGE[2][MOTOR_COUNT] = {
    {-90.0f, -90.0f, -90.0f, -90.0f, -90.0f, -90.0f, -90.0f,
        -90.0f, -90.0f, -90.0f, -90.0f, -90.0f, -90.0f, -90.0f,
        -90.0f, -90.0f, -90.0f, -90.0f, -90.0f, -90.0f},
    {90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f,
        90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f}};

/* kpkd
 *      Currently only use the maximum
 */
const float QKP_RANGE[2][MOTOR_COUNT] = {
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    {100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f,
        100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f,
        100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f}};
const float QKD_RANGE[2][MOTOR_COUNT] = {
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    {10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f,
        10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f}};

/* Config command constants */
const char *config_code_to_name[16] = {
    "UNDEFINED",    /* ox00 */
    "ACCLERTATION", /* 0x01 */
    "UNDEFINED",    /* 0x02 */
    "UNDEFINED",    /* 0x03 */
    "KT",           /* 0x04 */
    "KP",           /* 0x05 */
    "KD",           /* 0x06 */
    "POSITION",     /* 0x07 */
    "SPEED",        /* 0x08 */
    "TORQUE",       /* 0x09 */
    "CURRENT",      /* 0x0a */
    "UNDEFINED",    /* 0x0b */
    "UNDEFINED",    /* 0x0c */
    "UNDEFINED",    /* 0x0d */
    "UNDEFINED",    /* 0x0e */
    "UNDEFINED",    /* 0x0f */
};
const uint8_t CONFIG_ACCLERATION    = 0x01;
const uint8_t CONFIG_KT             = 0x04;
const uint8_t CONFIG_KP_RANGE       = 0x05;
const uint8_t CONFIG_KD_RANGE       = 0x06;
const uint8_t CONFIG_POS_RANGE      = 0x07;
const uint8_t CONFIG_VEL_RANGE      = 0x08;
const uint8_t CONFIG_TOR_RANGE      = 0x09;
const uint8_t CONFIG_CUR_RANGE      = 0x0a;

const uint8_t QUERY_KP_RANGE        = 23;
const uint8_t QUERY_KD_RANGE        = 24;
