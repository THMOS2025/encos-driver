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

/* limits - it is weired that the sdk uses relative */
static uint16_t kp_range[2][MOTOR_COUNT];
static uint16_t kd_range[2][MOTOR_COUNT];
static uint16_t pos_range[2][MOTOR_COUNT];
static uint16_t vel_range[2][MOTOR_COUNT];
static uint16_t tor_range[2][MOTOR_COUNT];
static uint16_t cur_range[2][MOTOR_COUNT];

/* encos status */
static uint8_t  channel_available[CHANNEL_COUNT];
static uint8_t  motor_to_channel[MOTOR_COUNT];
static uint8_t  motor_error[MOTOR_COUNT];
static uint16_t desired_pos_raw[MOTOR_COUNT];
static uint16_t current_pos_raw[MOTOR_COUNT];
static uint16_t current_vel_raw[MOTOR_COUNT];
static uint16_t current_tor_raw[MOTOR_COUNT];


/* 
 * Send command
 */

int send_query_id(const uint8_t channel)
{
    log_trace("Channel %hu: send query_id command", channel);
    return write_can_message(channel, 0x7ff, 4, 0xffff008200000000ull);
}

static int send_set_id(const uint8_t channel, const uint8_t old_id, const uint8_t new_id)
{
    log_debug("Channel %hu: send set_id command old=%hu new=%hu", \
            channel, old_id, new_id);
    return write_can_message(channel, 0x7ff, 6, ((uint64_t)old_id << 48ull) \
                                                | ((uint64_t)new_id << 16ull) \
                                                | 0x0000000400000000ull);
}

static int send_set_zero(const uint8_t channel, const uint8_t id)
{
    log_debug("Channel %hu: send set_zero command id=%hu", channel, id);
    return write_can_message(channel, 0x7ff, 4, ((uint64_t)id << 48ull) \
                                                | 0x0000000300000000ull);
}

static int send_pos_control(const uint8_t channel, const uint8_t id)
{
    return write_can_message(channel, id, 8, 0x0ffffull << 40ull \
                | ((uint64_t)(desired_pos_raw[id]) << 24ull) \
                | 0x7ff7ffull /* 7ff stands for zero in 0-4095 range */);
}

static int send_range_config(const uint8_t channel, const uint8_t id, \
        const uint8_t cfg, const uint16_t minn, const uint16_t maxn)
{
    return write_can_message(channel, id, 6, \
            0x06ull << 61ull \
            | 0x09ull << 48ull \
            | (uint64_t)minn << 32ull \
            | (uint64_t)maxn << 32ull);
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
        if((recv_buf >> 40ull) != 0xffff01ull) return NOT_RELATED_MSG;
        recv_id = (uint8_t)((recv_buf >> 24ull) & 0xffull);
        motor_to_channel[recv_id] = recv_ch;
        log_trace("Motor %hu found on channel %hu", recv_id, recv_ch);
        return COMMAND_SUCCESS;
    }
    else if(recv_len == 4){
        if(recv_buf == 0x8080018000000000ull) {
            log_warn("Channel %hu failed query motor ids", recv_ch);
            return COMMAND_FAILED; /* query id failed */
        }
        recv_id = (uint8_t)((recv_buf >> 48ull) & 0xffull);
        switch((recv_buf >> 32ull) & 0xffffull) {
        case 0x0100ull:
            log_warn("Motor %hu failed something", recv_id);
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
    return NOT_RELATED_MSG; 
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
    /* torque/current field : uint12 */
    current_tor_raw[recv_id] = (uint16_t)((recv_buf >> 16ull) & 0xfffull);
    return COMMAND_SUCCESS;
}

static int parse_motor_set_range()
{
    if(recv_len != 7 || recv_id >= MOTOR_COUNT || (recv_buf >> 48ull) != 0xfffeull)
        return NOT_RELATED_MSG; /* not related */
    log_info("Config %s success on motor %hu", \
            config_code_to_name[(recv_buf>>40ull)&0xffull], recv_id);
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
    memset(current_tor_raw, 0, sizeof(current_tor_raw));
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
                    log_info("Found motor %hu on channel %hu", \
                            recv_id, recv_ch);
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

int send_motor_set_zero(const uint8_t id)
{
    if(motor_to_channel[id] && channel_available[motor_to_channel[id]])
        return send_set_zero(motor_to_channel[id], id);
    log_warn("Can not send set zero for motor %hu", id);
    return -1;
}

int send_motors_pos(const float qpos[])
{
    /* 0~65536 : -12.5rad~12.5rad */
    uint8_t ok_cnt = 0;
    const float scale = 65536.0f / 2.0f;
    const float offset = 1.0f;

    #pragma omp simd
    for(uint8_t j = 0; j < MOTOR_COUNT; ++j)
        desired_pos_raw[j] = (uint16_t)((qpos[j] + offset) * scale);

    for(uint8_t j = 0, i; j < MOTOR_COUNT; ++j) {
        if((i = motor_to_channel[j]) == 0) continue;
        if(channel_available[i] == 0) continue;
        if(send_pos_control(i, j)) continue;
        ++ok_cnt;
    }

    if(ok_cnt != MOTOR_COUNT) {
        log_warn("Only %hu motors send successfully", ok_cnt);
        return -1;
    }
    return 0;
}

int pull_motors_msg()
{
    for(uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
        while(read_next_msg(i) == 0) {
            if(parse_motor_status() <= 0) continue;
            if(parse_motor_id() <= 0) continue;
            if(parse_motor_set_range() <= 0) continue;
        }
    }
}

int get_motors_pos_vel(float qpos[], float qvel[])
{
    const float scale = 2.0f / 65536.0f;
    const float offset = 1.0f;

    #pragma omp simd
    for(uint8_t j = 0; j < MOTOR_COUNT; ++j) {
        qpos[j] = (float)(current_pos_raw[j]) * scale - offset;
        qvel[j] = (float)(current_vel_raw[j]) * scale - offset;
    }

    return 0; 
}

int get_motors_pos_vel_tor(float qpos[], float qvel[], float qtor[])
{
    const float scale = 2.0f / 65536.0f;
    const float offset = 1.0f;

    #pragma omp simd
    for(uint8_t j = 0; j < MOTOR_COUNT; ++j) {
        qpos[j] = (float)(current_pos_raw[j]) * scale - offset;
        qvel[j] = (float)(current_vel_raw[j]) * scale - offset;
        qtor[j] = (float)(current_tor_raw[j]) * scale - offset; // Using same scale for now
    }

    return 0; 
}

static int send_motor_set_range( \
        const float cfg_range[2][MOTOR_COUNT], 
        uint16_t self_range[2][MOTOR_COUNT], 
        const float scaler, const uint8_t code)
{
    uint8_t ok_cnt = 0;
    for(uint8_t j = 0, i; j < MOTOR_COUNT; ++j) {
        if((i = motor_to_channel[j]) == 0) continue;
        if(channel_available[i] == 0) continue;
        if(send_range_config(motor_to_channel[j], j, code, 
                (self_range[0][j] = (uint16_t)(cfg_range[0][j] * scaler)), /* minn */
                (self_range[1][j] = (uint16_t)(cfg_range[1][j] * scaler))  /* maxn */
            ) != 0) continue;
        ++ok_cnt;
    }
    if(ok_cnt != MOTOR_COUNT) {
        log_warn("Set %s: only %hu motors send successfully", \
                config_code_to_name[code], ok_cnt);  
        return -1;
    }
}

int send_motor_set_pos_range(const float qpos_range[2][MOTOR_COUNT]) {
    return send_motor_set_range(qpos_range, pos_range, 100.0f, CONFIG_POS_RANGE);
}
int send_motor_set_vel_range(const float qvel_range[2][MOTOR_COUNT]) {
    return send_motor_set_range(qvel_range, vel_range, 100.0f, CONFIG_VEL_RANGE);
}
int send_motor_set_tor_range(const float qtor_range[2][MOTOR_COUNT]) {
    return send_motor_set_range(qtor_range, tor_range, 10.0f, CONFIG_TOR_RANGE);
}
int send_motor_set_cur_range(const float qcur_range[2][MOTOR_COUNT]) {
    return send_motor_set_range(qcur_range, cur_range, 10.0f, CONFIG_TOR_RANGE);
}
int send_motor_set_kp_range(const float qkp_range[2][MOTOR_COUNT]) {
    return send_motor_set_range(qkp_range, kp_range, 1.0f, CONFIG_KP_RANGE);
}
int send_motor_set_kd_range(const float qkd_range[2][MOTOR_COUNT]) {
    return send_motor_set_range(qkd_range, kd_range, 1.0f, CONFIG_KD_RANGE);
}
