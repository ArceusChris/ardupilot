/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */
/*
  DM-IMU serial external IMU backend.
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_DMIMU_ENABLED

#include "AP_ExternalAHRS_backend.h"

class AP_ExternalAHRS_DMIMU : public AP_ExternalAHRS_backend
{
public:
    AP_ExternalAHRS_DMIMU(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    int8_t get_port(void) const override;
    const char* get_name() const override;

    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void update() override;
    uint8_t num_gps_sensors(void) const override { return 0; }

private:
    static constexpr uint8_t FRAME_LEN = 19;
    static constexpr uint8_t FRAME_HEADER1 = 0x55;
    static constexpr uint8_t FRAME_HEADER2 = 0xAA;
    static constexpr uint8_t FRAME_TAIL = 0x0A;
    static constexpr uint8_t RID_ACCEL = 0x01;
    static constexpr uint8_t RID_GYRO = 0x02;
    static constexpr uint8_t RID_EULER = 0x03;
    static constexpr uint16_t DEVICE_OUTPUT_RATE_HZ = 500;
    static constexpr uint16_t EKF_OUTPUT_RATE_HZ = 200;
    static constexpr uint32_t EKF_OUTPUT_PERIOD_US = 1000000UL / EKF_OUTPUT_RATE_HZ;
    static constexpr uint8_t SAMPLE_BUFFER_SIZE = 32;
    static constexpr float MAX_ACCEL_MSS = 100.0f;
    static constexpr float MIN_ACCEL_LENGTH_MSS = 0.0f;
    static constexpr float MAX_GYRO_RADS = 20.0f;

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t baudrate = 0;
    int8_t port_num = -1;
    bool setup_complete = false;
    bool thread_started = false;

    uint8_t read_buffer[128];
    uint8_t frame[FRAME_LEN];
    uint8_t frame_ofs = 0;

    struct sample_t {
        Vector3f accel;
        Vector3f gyro;
        uint64_t time_us;
    };

    struct {
        mutable HAL_Semaphore semaphore;
        Vector3f pending_accel;
        Vector3f pending_gyro;
        sample_t samples[SAMPLE_BUFFER_SIZE];
        uint64_t last_accel_us = 0;
        uint64_t last_gyro_us = 0;
        uint64_t next_publish_us = 0;
        uint8_t sample_head = 0;
        uint8_t sample_count = 0;
        bool have_accel = false;
        bool have_gyro = false;
        bool pending_have_accel = false;
        bool pending_have_gyro = false;
    } driver_state;

    void update_thread();
    bool check_uart();
    void configure_device();
    void send_command(const uint8_t *command, uint8_t len);
    uint16_t output_interval_ms() const;
    void parse_byte(uint8_t b, uint64_t now_us);
    bool parse_frame(Vector3f &value, uint8_t &rid) const;
    bool valid_sensor_sample(uint8_t rid, const Vector3f &value) const;
    void handle_frame(uint8_t rid, const Vector3f &value, uint64_t now_us);
    void push_sample_locked(const Vector3f &accel, const Vector3f &gyro, uint64_t time_us);
    bool pop_closest_sample(uint64_t target_us, sample_t &sample);
    void publish_due();
    void publish_sample(const sample_t &sample);

    static uint16_t crc16(const uint8_t *data, uint8_t len);
    static float le_float(const uint8_t *data);
};

#endif // AP_EXTERNAL_AHRS_DMIMU_ENABLED
