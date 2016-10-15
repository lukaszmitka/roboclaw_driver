//
// Created by lukasz on 30.09.16.
//

#ifndef HERKULEX_SERVO_DRIVER_COMPORTDRIVER_H
#define HERKULEX_SERVO_DRIVER_COMPORTDRIVER_H

#include <string>
#include <vector>
#include <iostream>
#include "rs232.h"

class ComPortDriver {
public:
    ComPortDriver(int port_number);

    ~ComPortDriver();

    std::vector<char> read_data();
    std::vector<char> read_data(bool wait_for_end_of_line);

    int send_data(std::vector<char> data);

    bool is_port_opened(){return port_opened;};
private:
    bool port_opened = false;
    int port_num;

    int init_comport(int port_number, long baud_rate);

    int close_comport(int port_number);

};


#endif //HERKULEX_SERVO_DRIVER_COMPORTDRIVER_H
