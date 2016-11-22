//
// Created by lukasz on 16.10.16.
//

#ifndef ROBOCLAW_DRIVER_ROBOCLAW_H
#define ROBOCLAW_DRIVER_ROBOCLAW_H


#include <string>
#include "ComPortDriver.h"

class Roboclaw {

    // constructors & destructors
public:
    /**
     * Left motor is to be connected to M1 output on Roboclaw board
     */
    Roboclaw(u_char address, int port_number, uint32_t pulses_per_meter);

    ~Roboclaw();

    // fields
private:
    std::string version;
    std::vector<u_char> l_data;
    u_char device_address;
    //int com_port_number;
    ComPortDriver *comPortDriver;
    uint16_t crc;
    uint32_t ppm; // encoder pulses per meter
    int32_t left_target_pps = 0; // target pulses per second for left wheel
    int32_t right_target_pps = 0; // target pulses per second for right wheel
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

    void clear_crc();

    void add_crc();

    uint16_t add_byte(uint16_t crc, u_char byte);

private:
    bool comPort_opened;
    void driveM1SignedSpeed(int32_t speed);

    void driveM2SignedSpeed(int32_t speed);

    void read_version();

    uint16_t crc_update(uint16_t crc_val, u_char byte);
};

#endif //ROBOCLAW_DRIVER_ROBOCLAW_H
