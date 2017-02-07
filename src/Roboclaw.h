//
// Created by lukasz on 16.10.16.
//

#ifndef ROBOCLAW_DRIVER_ROBOCLAW_H
#define ROBOCLAW_DRIVER_ROBOCLAW_H

#define REPLY_SIZE_ACK 1

#define COM_READ_VERSION 21
#define COM_READ_VERSION_REPLY_SIZE 50

#define COM_DRV_M1_SIGNED_SPD 35
#define COM_DRV_M1_SIGNED_SPD_REPLY_SIZE REPLY_SIZE_ACK
#define COM_DRV_M2_SIGNED_SPD 36
#define COM_DRV_M2_SIGNED_SPD_REPLY_SIZE REPLY_SIZE_ACK
#define COM_DRV_M1_SIGNED_SPD_ACCEL 38
#define COM_DRV_M1_SIGNED_SPD_ACCEL_REPLY_SIZE REPLY_SIZE_ACK
#define COM_DRV_M2_SIGNED_SPD_ACCEL 39
#define COM_DRV_M2_SIGNED_SPD_ACCEL_REPLY_SIZE REPLY_SIZE_ACK


#define COM_READ_ENCODER_M1 16
#define COM_READ_ENCODER_M1_REPLY_SIZE 7 // enc1 (4bytes), status, crc
#define COM_READ_ENCODER_M2 17
#define COM_READ_ENCODER_M2_REPLY_SIZE 7 // enc1 (4bytes), status, crc
#define COM_READ_M1_SPD_1SEC 18
#define COM_READ_M1_SPD_1SEC_REPLY_SIZE 7 // speed (4bytes), status, crc
#define COM_READ_M2_SPD_1SEC 19
#define COM_READ_M2_SPD_1SEC_REPLY_SIZE 7 // speed (4bytes), status, crc
#define COM_RESET_ENCODERS 20
#define COM_RESET_ENCODERS_REPLY_SIZE REPLY_SIZE_ACK
#define COM_READ_RAW_SPD_M1 30
#define COM_READ_RAW_SPD_M1_REPLY_SIZE 7 // speed (4bytes), status, crc
#define COM_READ_RAW_SPD_M2 31
#define COM_READ_RAW_SPD_M2_REPLY_SIZE 7 // speed (4bytes), status, crc
#define COM_READ_ENCODER_COUNTERS 78
#define COM_READ_ENCODER_COUNTERS_REPLY_SIZE 10

#include <string>
#include "serial/serial.h"
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <stdlib.h>

class Roboclaw {

    // constructors & destructors
public:
    /**
     * Left motor is to be connected to M1 output on Roboclaw board
     */
    Roboclaw(u_char address, int port_number, uint32_t pulses_per_meter);

    Roboclaw(u_char address, std::string port, uint32_t pulses_per_meter, uint16_t timeout);

    ~Roboclaw();

    // fields
private:
    serial::Serial *my_serial;
    std::string version;
    std::vector<u_char> l_data;
    u_char device_address;
    //int com_port_number;
    //ComPortDriver *comPortDriver;
    uint16_t crc;
    uint32_t ppm; // encoder pulses per meter
    int64_t left_target_pps = 0; // target pulses per second for left wheel
    int64_t right_target_pps = 0; // target pulses per second for right wheel
    u_char tmp;

    // functions
public:
    bool read_encoders(long *enc1, long *enc2);

    bool has_acces_to_ComPort();

    /**
     * left_motor - desired left wheel linear speed in meters per second
     * right_motor - desired right wheel linear speed in meters per second
     */
    bool set_speed(double left_motor, double right_motor);

    bool set_speed_with_accel(double left_motor, double right_motor, uint32_t accel);

    void clear_crc();

    void add_crc();

    uint16_t add_byte(uint16_t crc, u_char byte);

private:
    bool comPort_opened;

    bool driveM1SignedSpeed(int32_t speed);

    bool driveM2SignedSpeed(int32_t speed);

    bool driveM1SignedSpeedAccel(int32_t speed, uint32_t accel);

    bool driveM2SignedSpeedAccel(int32_t speed, uint32_t accel);

    bool read_version();

    bool read_version(bool print_version);

    bool reset_encoder_counters();

    uint16_t crc_update(uint16_t crc_val, u_char byte);

    bool execute_command(std::vector<u_char> data);

    bool execute_command(u_char device_address, u_char command, u_char reply_size,
                         std::vector<u_char> &response);

    //std::vector<u_char> execute_command(u_char device_address, u_char command, u_char reply_size);

    //std::vector<u_char> execute_command(std::vector<u_char> data, u_char reply_size);
};

#endif //ROBOCLAW_DRIVER_ROBOCLAW_H
