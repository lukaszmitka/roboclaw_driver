//
// Created by lukasz on 16.10.16.
//

#include "Roboclaw.h"

Roboclaw::Roboclaw(u_char address, int port_number, uint32_t pulses_per_meter) {
   ppm = pulses_per_meter;
   //std::cout << "Get ComPortDriver" << std::endl;
   comPortDriver = new ComPortDriver(port_number);
   device_address = address;
   //std::cout << "Read roboclaw version" << std::endl;
   if (comPortDriver->is_port_opened()) {
      comPort_opened = true;
      read_version();
   } else {
      comPort_opened = false;
      //std::cout << "Can not open ComPort" << std::endl;
   }
}

bool Roboclaw::has_acces_to_ComPort() {
   return comPort_opened;
}

Roboclaw::~Roboclaw() {
   comPortDriver->flush_input();
   comPortDriver->~ComPortDriver();
}

uint16_t Roboclaw::crc_update(uint16_t crc_val, u_char byte) {
   //std::cout << "input: crc=" << (uint) crc_val << " data=" << (uint) byte;
   crc_val = crc_val ^ ((u_int16_t) byte << 8);
   //std::cout << " XOR=" << (uint) crc_val;
   for (unsigned char bit = 0; bit < 8; bit++) {
      if (crc_val & 0x8000) {
         crc_val = (crc_val << 1) ^ 0x1021;
         //std::cout << " '^'=";
      } else {
         crc_val = crc_val << 1;
      }
      //std::cout << ", " << (uint) crc_val;
   }
   //std::cout << std::endl;
   return crc_val;
}

uint16_t Roboclaw::add_byte(uint16_t crc, u_char byte) {
   l_data.push_back(byte);
   crc = crc_update(crc, byte);
   return crc;
}

void Roboclaw::clear_crc() {
   crc = 0;
   l_data.clear();
}

void Roboclaw::add_crc() {
   uint8_t crc_h = crc >> 8;
   uint8_t crc_l = crc & 0xFF;
   //std::cout << "CRC_h: " << (uint) crc_h << " CRC_l: " << (uint) crc_l << std::endl;
   l_data.push_back(crc_h);
   l_data.push_back(crc_l);
}

bool Roboclaw::set_speed(double left_motor, double right_motor) {
   //std::cout << "target spd[m/s] = " << left_motor << ", " << right_motor;
   left_target_pps = (int32_t) ((left_motor) * ppm);
   right_target_pps = (int32_t) ((right_motor) * ppm);
   //std::cout << "target spd [pps]= " << left_target_pps << ", " << right_target_pps << std::endl;
   driveM1SignedSpeed(left_target_pps);
   driveM2SignedSpeed(right_target_pps);
   return true;
}

void Roboclaw::driveM1SignedSpeed(int32_t speed) {
   clear_crc();
   crc = add_byte(crc, device_address); // address
   crc = add_byte(crc, 35); //command Drive M1 With Signed Speed
   // 4 bytes of data
   tmp = (speed >> 24) & 0xFF;
   crc = add_byte(crc, tmp);
   tmp = (speed >> 16) & 0xFF;
   crc = add_byte(crc, tmp);
   tmp = (speed >> 8) & 0xFF;
   crc = add_byte(crc, tmp);
   tmp = (speed >> 0) & 0xFF;
   crc = add_byte(crc, tmp);
   add_crc();
   comPortDriver->flush_input();
   comPortDriver->send_data(l_data);
   l_data.clear();
   //std::cout << "read port" << std::endl;
   l_data = comPortDriver->read_data(1);
   //std::cout << "received data: " << (int) (uint8_t) l_data[0] << std::endl;
}

void Roboclaw::driveM2SignedSpeed(int32_t speed) {
   clear_crc();
   crc = add_byte(crc, device_address); // address
   crc = add_byte(crc, 36); //command Drive M2 With Signed Speed
   // 4 bytes of data
   tmp = (speed >> 24) & 0xFF;
   crc = add_byte(crc, tmp);
   tmp = (speed >> 16) & 0xFF;
   crc = add_byte(crc, tmp);
   tmp = (speed >> 8) & 0xFF;
   crc = add_byte(crc, tmp);
   tmp = (speed >> 0) & 0xFF;
   crc = add_byte(crc, tmp);
   add_crc();
   comPortDriver->flush_input();
   comPortDriver->send_data(l_data);
   l_data.clear();
   //std::cout << "read port" << std::endl;
   l_data = comPortDriver->read_data(1);
   //std::cout << "received data: " << (int) (uint8_t) l_data[0] << std::endl;
}

void Roboclaw::read_version() {
   std::vector<u_char> data;
   std::string ver;
   int i;

   data.push_back(device_address);
   data.push_back(21);
   comPortDriver->send_data(data);
   data.clear();
   data = comPortDriver->read_data(true);
   for (i = 0; i < data.size(); i++) {
      ver.insert(i, 1, data[i]);
   }
   version = ver;
   return;
}

bool Roboclaw::read_encoders(long *enc1, long *enc2) {
   std::vector<u_char> data;

   data.push_back(0x80);
   data.push_back(78);
   comPortDriver->flush_input();
   comPortDriver->send_data(data);
   data.clear();
   data = comPortDriver->read_data(10);

   unsigned long encoder_1 = ((u_char) data[0]) << 24 | ((u_char) data[1] << 16) | ((u_char)
         data[2]) << 8 | ((u_char) data[3]);
   *enc1 = encoder_1;
   unsigned long encoder_2 = ((u_char) data[4]) << 24 | ((u_char) data[5] << 16) | ((u_char)
         data[6]) << 8 | ((u_char) data[7]);
   *enc2 = encoder_2;
   //std::cout << "Enc1: " << encoder_1 << " Enc2: " << encoder_2 << std::endl;

}