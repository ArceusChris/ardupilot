/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */
/*
  DM-IMU serial external IMU backend.
 */

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_DMIMU_ENABLED

#include "AP_ExternalAHRS_DMIMU.h"

#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

#include <math.h>
#include <string.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_DMIMU::AP_ExternalAHRS_DMIMU(AP_ExternalAHRS *_frontend, AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart || baudrate == 0 || port_num == -1) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DM-IMU: serial port not found");
        return;
    }

    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::IMU));

    thread_started = hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_DMIMU::update_thread, void),
                                                  "AHRS_DMIMU", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0);
    if (!thread_started) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DM-IMU: failed to create thread");
    }
}

int8_t AP_ExternalAHRS_DMIMU::get_port(void) const
{
    return uart ? port_num : -1;
}

const char* AP_ExternalAHRS_DMIMU::get_name() const
{
    return "DM-IMU";
}

bool AP_ExternalAHRS_DMIMU::healthy(void) const
{
    const uint64_t now_us = AP_HAL::micros64();

    WITH_SEMAPHORE(driver_state.semaphore);
    return driver_state.have_accel &&
           driver_state.have_gyro &&
           now_us - driver_state.last_accel_us <= 200000U &&
           now_us - driver_state.last_gyro_us <= 200000U;
}

bool AP_ExternalAHRS_DMIMU::initialised(void) const
{
    return setup_complete;
}

bool AP_ExternalAHRS_DMIMU::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "DM-IMU unhealthy");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_DMIMU::update()
{
    if (!thread_started) {
        check_uart();
    }
    publish_due();
}

void AP_ExternalAHRS_DMIMU::update_thread()
{
    while (true) {
        const bool got_data = check_uart();
        publish_due();
        if (!got_data) {
            hal.scheduler->delay_microseconds(500);
        }
    }
}

bool AP_ExternalAHRS_DMIMU::check_uart()
{
    if (!uart) {
        return false;
    }

    if (!setup_complete) {
        uart->begin(baudrate);
        configure_device();
        setup_complete = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "DM-IMU: init baud:%u imu:%uHz ekf:%uHz",
                      unsigned(baudrate),
                      unsigned(DEVICE_OUTPUT_RATE_HZ),
                      unsigned(EKF_OUTPUT_RATE_HZ));
    }

    const auto nread = uart->read(read_buffer, sizeof(read_buffer));
    if (nread <= 0) {
        return false;
    }

    const uint64_t now_us = AP_HAL::micros64();
    for (ssize_t i = 0; i < nread; i++) {
        parse_byte(read_buffer[i], now_us);
    }

    return true;
}

void AP_ExternalAHRS_DMIMU::configure_device()
{
    static const uint8_t enter_setting_mode[] {0xAA, 0x06, 0x01, 0x0D};
    static const uint8_t enable_accel[] {0xAA, 0x01, 0x14, 0x0D};
    static const uint8_t enable_gyro[] {0xAA, 0x01, 0x15, 0x0D};
    static const uint8_t disable_euler[] {0xAA, 0x01, 0x06, 0x0D};
    static const uint8_t disable_quaternion[] {0xAA, 0x01, 0x07, 0x0D};
    static const uint8_t enter_normal_mode[] {0xAA, 0x06, 0x00, 0x0D};

    const uint16_t interval_ms = output_interval_ms();
    const uint8_t set_output_interval[] {
        0xAA,
        0x02,
        uint8_t(interval_ms & 0xFF),
        uint8_t(interval_ms >> 8),
        0x0D,
    };

    send_command(enter_setting_mode, sizeof(enter_setting_mode));
    send_command(enable_accel, sizeof(enable_accel));
    send_command(enable_gyro, sizeof(enable_gyro));
    send_command(disable_euler, sizeof(disable_euler));
    send_command(disable_quaternion, sizeof(disable_quaternion));
    send_command(set_output_interval, sizeof(set_output_interval));
    send_command(enter_normal_mode, sizeof(enter_normal_mode));
    uart->discard_input();
}

void AP_ExternalAHRS_DMIMU::send_command(const uint8_t *command, uint8_t len)
{
    for (uint8_t i = 0; i < 5; i++) {
        uart->write(command, len);
        uart->flush();
        hal.scheduler->delay(10);
    }
}

uint16_t AP_ExternalAHRS_DMIMU::output_interval_ms() const
{
    return 1000U / DEVICE_OUTPUT_RATE_HZ;
}

void AP_ExternalAHRS_DMIMU::parse_byte(uint8_t b, uint64_t now_us)
{
    if (frame_ofs == 0 && b != FRAME_HEADER1) {
        return;
    }

    if (frame_ofs == 1 && b != FRAME_HEADER2) {
        frame_ofs = (b == FRAME_HEADER1) ? 1 : 0;
        frame[0] = FRAME_HEADER1;
        return;
    }

    frame[frame_ofs++] = b;
    if (frame_ofs < FRAME_LEN) {
        return;
    }

    frame_ofs = 0;

    Vector3f value;
    uint8_t rid;
    if (parse_frame(value, rid)) {
        handle_frame(rid, value, now_us);
    }
}

bool AP_ExternalAHRS_DMIMU::parse_frame(Vector3f &value, uint8_t &rid) const
{
    if (frame[0] != FRAME_HEADER1 ||
        frame[1] != FRAME_HEADER2 ||
        frame[FRAME_LEN - 1] != FRAME_TAIL) {
        return false;
    }

    rid = frame[3];
    if (rid != RID_ACCEL && rid != RID_GYRO && rid != RID_EULER) {
        return false;
    }

    const uint16_t crc_wire = uint16_t(frame[16]) | (uint16_t(frame[17]) << 8);
    const uint16_t crc_with_header = crc16(frame, 16);
    const uint16_t crc_without_header = crc16(&frame[2], 14);
    if (crc_wire != crc_with_header && crc_wire != crc_without_header) {
        return false;
    }

    value.x = le_float(&frame[4]);
    value.y = le_float(&frame[8]);
    value.z = le_float(&frame[12]);

    return isfinite(value.x) && isfinite(value.y) && isfinite(value.z);
}

void AP_ExternalAHRS_DMIMU::handle_frame(uint8_t rid, const Vector3f &value, uint64_t now_us)
{
    if (!valid_sensor_sample(rid, value)) {
        return;
    }

    WITH_SEMAPHORE(driver_state.semaphore);

    if (rid == RID_ACCEL) {
        driver_state.pending_accel = value;
        driver_state.pending_have_accel = true;
        driver_state.have_accel = true;
        driver_state.last_accel_us = now_us;
    } else if (rid == RID_GYRO) {
        driver_state.pending_gyro = value;
        driver_state.pending_have_gyro = true;
        driver_state.have_gyro = true;
        driver_state.last_gyro_us = now_us;
    }

    if (driver_state.pending_have_accel && driver_state.pending_have_gyro) {
        uint64_t sample_time_us = driver_state.last_accel_us;
        if (driver_state.last_gyro_us > sample_time_us) {
            sample_time_us = driver_state.last_gyro_us;
        }
        push_sample_locked(driver_state.pending_accel, driver_state.pending_gyro, sample_time_us);
        driver_state.pending_have_accel = false;
        driver_state.pending_have_gyro = false;
    }
}

void AP_ExternalAHRS_DMIMU::push_sample_locked(const Vector3f &accel, const Vector3f &gyro, uint64_t time_us)
{
    driver_state.samples[driver_state.sample_head] = {accel, gyro, time_us};
    driver_state.sample_head = (driver_state.sample_head + 1) % SAMPLE_BUFFER_SIZE;
    if (driver_state.sample_count < SAMPLE_BUFFER_SIZE) {
        driver_state.sample_count++;
    }
}

bool AP_ExternalAHRS_DMIMU::pop_closest_sample(uint64_t target_us, sample_t &sample)
{
    WITH_SEMAPHORE(driver_state.semaphore);

    if (driver_state.sample_count == 0) {
        return false;
    }

    const uint8_t oldest = (driver_state.sample_head + SAMPLE_BUFFER_SIZE - driver_state.sample_count) % SAMPLE_BUFFER_SIZE;
    uint8_t best_offset = 0;
    uint64_t best_delta = UINT64_MAX;

    for (uint8_t i = 0; i < driver_state.sample_count; i++) {
        const uint8_t idx = (oldest + i) % SAMPLE_BUFFER_SIZE;
        const uint64_t sample_time_us = driver_state.samples[idx].time_us;
        const uint64_t delta = (sample_time_us > target_us) ? sample_time_us - target_us : target_us - sample_time_us;
        if (delta < best_delta) {
            best_delta = delta;
            best_offset = i;
        }
    }

    const uint8_t best_idx = (oldest + best_offset) % SAMPLE_BUFFER_SIZE;
    sample = driver_state.samples[best_idx];
    driver_state.sample_count -= best_offset + 1;
    return true;
}

void AP_ExternalAHRS_DMIMU::publish_due()
{
    const uint64_t now_us = AP_HAL::micros64();
    uint64_t target_us;

    {
        WITH_SEMAPHORE(driver_state.semaphore);
        if (driver_state.next_publish_us == 0) {
            driver_state.next_publish_us = now_us + EKF_OUTPUT_PERIOD_US;
            return;
        }
        if (now_us < driver_state.next_publish_us) {
            return;
        }
        if (now_us - driver_state.next_publish_us > 2U * EKF_OUTPUT_PERIOD_US) {
            driver_state.next_publish_us = now_us;
        }
        target_us = driver_state.next_publish_us;
        driver_state.next_publish_us += EKF_OUTPUT_PERIOD_US;
    }

    sample_t sample;
    if (pop_closest_sample(target_us, sample)) {
        publish_sample(sample);
    }
}

void AP_ExternalAHRS_DMIMU::publish_sample(const sample_t &sample)
{
    AP_ExternalAHRS::ins_data_message_t ins {};
    ins.accel = sample.accel;
    ins.gyro = sample.gyro;
    ins.temperature = 0.0f;

    {
        WITH_SEMAPHORE(state.sem);
        state.accel = ins.accel;
        state.gyro = ins.gyro;
    }

    AP::ins().handle_external(ins);
}

bool AP_ExternalAHRS_DMIMU::valid_sensor_sample(uint8_t rid, const Vector3f &value) const
{
    if (!isfinite(value.x) || !isfinite(value.y) || !isfinite(value.z)) {
        return false;
    }

    if (rid == RID_ACCEL) {
        return fabsf(value.x) <= MAX_ACCEL_MSS &&
               fabsf(value.y) <= MAX_ACCEL_MSS &&
               fabsf(value.z) <= MAX_ACCEL_MSS &&
               value.length() >= MIN_ACCEL_LENGTH_MSS;
    }

    if (rid == RID_GYRO) {
        return fabsf(value.x) <= MAX_GYRO_RADS &&
               fabsf(value.y) <= MAX_GYRO_RADS &&
               fabsf(value.z) <= MAX_GYRO_RADS;
    }

    return true;
}

uint16_t AP_ExternalAHRS_DMIMU::crc16(const uint8_t *data, uint8_t len)
{
    static const uint16_t crc16_table[256] {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
        0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
        0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
        0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
        0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
        0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
        0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
        0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
        0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
        0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
        0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
        0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
        0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
        0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
        0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
        0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
        0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
        0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
        0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
        0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
        0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
        0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
    };

    uint16_t crc = 0xFFFF;
    while (len--) {
        const uint8_t index = (crc >> 8) ^ *data++;
        crc = (crc << 1) ^ crc16_table[index];
    }
    return crc;
}

float AP_ExternalAHRS_DMIMU::le_float(const uint8_t *data)
{
    const uint32_t u = uint32_t(data[0]) |
                       (uint32_t(data[1]) << 8) |
                       (uint32_t(data[2]) << 16) |
                       (uint32_t(data[3]) << 24);
    float ret;
    memcpy(&ret, &u, sizeof(ret));
    return ret;
}

#endif // AP_EXTERNAL_AHRS_DMIMU_ENABLED
