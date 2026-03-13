#include <stdint.h>
#include "log.h"

const float PAI = 3.1415926535897;

/* return definitions */
/* TODO: isolated each error */
const int COMMAND_SUCCESS = 0;
const int COMMAND_FAILED = -1;
const int NOT_RELATED_MSG = 2;

/* configurations */
const uint8_t       MAX_SEND_FAILED         = 100;    /* after 100 send failed, consider motor loss */
const uint32_t      INITIAL_SCAN_MOTOR_TIME = 4;

const int LOG_LEVEL         = LOG_TRACE;
