/* driver.c
 *  high-level abstract ready for use
 */

#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

#include "driver.h"
#include "constant.h"
#include "encos_command.h"


int initialize_driver()
{
    initialize_motors();
    scan_motors(1000000);
}

int uninitialize_driver()
{
    uninitialize_motors();
}
