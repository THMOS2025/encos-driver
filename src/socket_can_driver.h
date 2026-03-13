/* can_driver.h
 *      Provide an abstract of can communication
 */
#ifndef _H_SOCKET_CAN_DRIVER
#define _H_SOCKET_CAN_DRIVER

#include <stddef.h>
#include <stdint.h>

#include "constant.h"


/**
 * @brief Initializes the SocketCAN Channel and sets it to non-blocking mode.
 * @param channel The logical channel index (0 to CAN_MAX_CHANNELS - 1).
 * @return 0 on success, -1 on failure. 1 if already initialized
 */
int initialize_can(const uint8_t channel);


/**
 * @brief Uninitializes the SocketCAN Channel.
 * @param channel The logical channel index.
 * @return 0 on success, -1 on failure.
 */
int uninitialize_can(const uint8_t channel);


/**
 * @brief Transmits a single CAN message.
 * @param channel The logical channel index.
 * @param id The CAN ID.
 * @param len The data length (DLC, 0-8).
 * @param data The 64-bit data payload.
 * @return 0 on success, -1 on failure.
 */
int
write_can_message(const uint8_t channel, 
        const uint16_t id, 
        const size_t len, 
        const uint64_t data);

/**
 * @brief Reads a single CAN message from the receive buffer (non-blocking).
 * @param channel The logical channel index.
 * @param id Pointer to store the received CAN ID.
 * @param len Pointer to store the received data length (DLC).
 * @param data Pointer to store the received 64-bit data payload.
 * @return 0 on success, 1 if no message is available (non-blocking), -1 on error.
 */
int
read_can_message(const uint8_t channel, 
        uint16_t *id,
        size_t *len,
        uint64_t *data);

#endif
