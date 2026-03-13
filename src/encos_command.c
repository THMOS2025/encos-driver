/* encos_command.h
 *      wrapping binaray encos commands into apis
 */

#include <time.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "encos_command.h"
#include "socket_can_driver.h"
#include "constant.h"
#include "log.h"

static uint8_t  recv_ch;
static uint64_t recv_buf;
static size_t   recv_len;
static uint16_t recv_id;

static uint8_t  channel_available[CHANNEL_COUNT];
static uint8_t  motor_to_channel[MOTOR_COUNT];
static uint8_t  motor_error[MOTOR_COUNT];
static uint16_t desired_pos_raw[MOTOR_COUNT];
static uint16_t current_pos_raw[MOTOR_COUNT];
static uint16_t current_vel_raw[MOTOR_COUNT];


/* 
 * Send command
 */

int send_query_id(const uint8_t channel)
{
    return write_can_message(channel, 0x7ff, 4, 0xffff008200000000ull);
}

static int send_set_id(const uint8_t channel, const uint8_t old_id, const uint8_t new_id)
{
    return write_can_message(channel, 0x7ff, 6, ((uint64_t)old_id << 48ull) \
                                                | ((uint64_t)new_id << 16ull) \
                                                | 0x0000000400000000ull);
}

static int send_pos_control(const uint8_t channel, const uint8_t id)
{
    return write_can_message(channel, id, 8, 0x1FFFFFull << 40ull \
                | ((uint64_t)(desired_pos_raw[id]) << 24ull));
}



/*
 * Parse response
 */

static int read_next_msg(const uint8_t channel)
{
    /* Left aligned */
    recv_ch = channel;
    return read_can_message(channel, &recv_id, &recv_len, &recv_buf);
}

static int parse_motor_id()
{
    if(recv_id != 0x7ff) return NOT_RELATED_MSG;
    if(recv_len == 5) { /* Query ID success */
        if((recv_buf >> 40ull) != 0xffff00ull) return NOT_RELATED_MSG;
        recv_id = (uint8_t)((recv_buf >> 24ull) & 0xffull);
        motor_to_channel[recv_id] = recv_ch;
        log_trace("Motor %hu found on channel %hu", recv_id, recv_ch);
        return COMMAND_SUCCESS;
    }
    else{
        if(recv_buf == 0x8080018000000000ull) {
            log_debug("Channel %hu failed query motor ids", recv_ch);
            return COMMAND_FAILED; /* query id failed */
        }
        recv_id = (uint8_t)((recv_buf >> 48ull) & 0xffull);
        switch((recv_buf >> 32ull) & 0xffffull) {
        case 0x0100ull:
            log_debug("Motor %hu failed something", recv_id);
            return COMMAND_FAILED; /* set failed (section 5 of the manual) */
        case 0x0103ull:
            log_trace("Motor %hu set zero point success", recv_id);
            return COMMAND_SUCCESS; /* set success */
        case 0x0104ull:
            log_trace("A motor change id to %hu on channel %hu", recv_id, recv_ch);
            return COMMAND_SUCCESS; /* change id success  */
        default:
            return NOT_RELATED_MSG; 
        }
    }
}

static int parse_motor_status() /* We only use response class 1 */
{
    /* response type field: uint3 */
    if(recv_len != 8 || recv_id >= MOTOR_COUNT || (recv_buf >> 61ull) != 0x01ull)
        return NOT_RELATED_MSG; /* not related */
    /* error field : uint5 */
    motor_error[recv_id] = ((recv_buf >> 56ull) & 0x1full);
    /* pos field : uint16 */
    current_pos_raw[recv_id] = (uint16_t)((recv_buf >> 40ull) & 0xffffull);
    /* omega field : uint12 */
    current_vel_raw[recv_id] = (uint16_t)((recv_buf >> 28ull) & 0xfffull);
    return COMMAND_SUCCESS;
}


/* 
 * Exposed APIs
 */
int initialize_motors()
{
    int ret = 0;
    memset(channel_available, 0, sizeof(channel_available));
    memset(desired_pos_raw, 0, sizeof(desired_pos_raw));
    memset(current_pos_raw, 0, sizeof(current_pos_raw));
    memset(current_vel_raw, 0, sizeof(current_vel_raw));
    for(uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
        if(initialize_can(i) < 0) continue;
        channel_available[i] = 1; 
    }
    return ret;
}

int uninitialize_motors()
{
    int ret = 0;
    for(uint8_t i = 0; i < CHANNEL_COUNT; ++i)
        if(channel_available[i])
            ret |= uninitialize_can(i);
    return ret;
}

int scan_motors(const uint32_t timeout_us)
{
    uint8_t total_motors = 0, scanned[MOTOR_COUNT];
    memset(scanned, 0, sizeof(scanned));
    for(uint64_t t = 0; t < timeout_us; t += 10000) {
        for(uint8_t i = 0; i < CHANNEL_COUNT; ++i)
            if(channel_available[i])
                send_query_id(i);
        usleep(10000);
        for(uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
            if(!channel_available[i])
                continue;
            while(read_next_msg(i) == 0) {
                if(parse_motor_id() == 0 && !scanned[recv_id]) {
                    ++total_motors;
                    scanned[recv_id] = 1;
                    motor_error[recv_id] = 0;
                }
            }
        }
        if(total_motors >= MOTOR_COUNT) {
            log_info("Found %hu motors.", total_motors);
            return 0;
        }
    }
    log_warn("Scan timeout after %u us, only found %hu motors", \
            timeout_us, total_motors);
    return -1; /* timeout */
}

int send_motors_pos(const float qpos[])
{
    /* 0~65536 : -12.5rad~12.5rad */
    uint8_t ok_cnt = 0;
    const float scale = 2621.44f;  /* precompute the constant helps trigger simd */
    const float offset = 12.5f;

    #pragma omp simd
    for(int j = 0; j < MOTOR_COUNT; ++j) {
        // Linear mapping: (qpos + 12.5) * 2621.44
        desired_pos_raw[j] = (uint16_t)((qpos[j] + offset) * scale);
    }

    for(int j = 0, i; j < MOTOR_COUNT; ++j) {
        if((i = motor_to_channel[j]) == 0) continue;
        if(channel_available[i] == 0) continue;
        if(send_pos_control(i, j)) continue;
        ++ok_cnt;
    }

    return (ok_cnt == MOTOR_COUNT) ? 0 : -1;
}

int pull_motors_msg()
{
    for(uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
        while(read_next_msg(i) == 0) {
            if(parse_motor_status() <= 0) continue;
            if(parse_motor_id() <= 0) continue;
        }
    }
}

int get_motors_pos_vel(float qpos[], float qvel[])
{
    const float pos_scale = 25.0f / 65536.0f;
    const float pos_offset = -12.5f;

    const float vel_scale = 36.0f / 4096.0f;
    const float vel_offset = -18.0f;

    #pragma omp simd
    for(int j = 0; j < MOTOR_COUNT; ++j) {
        qpos[j] = ((float)current_pos_raw[j] * pos_scale) + pos_offset;
        qvel[j] = ((float)current_vel_raw[j] * vel_scale) + vel_offset;
    }

    return 0; 
}
