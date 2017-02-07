//
// Created by lukasz on 16.10.16.
//

#include "Roboclaw.h"

Roboclaw::Roboclaw(u_char address, std::string port, uint32_t pulses_per_meter, uint16_t timeout) {
//std::cout << "Create Serial instance" << std::endl;
   my_serial = new serial::Serial(port, 115200, serial::Timeout::simpleTimeout(timeout));
   if (my_serial->isOpen()) {
      comPort_opened = true;
      ppm = pulses_per_meter;
      device_address = address;
      std::cout << "Read roboclaw version" << std::endl;
      read_version(true);
      reset_encoder_counters();
   } else {
      comPort_opened = false;
      std::cout << "Can not access comport" << std::endl;
   }

}

bool Roboclaw::has_acces_to_ComPort() {
   return comPort_opened;
}

Roboclaw::~Roboclaw() {
   my_serial->flush();
   my_serial->close();
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

bool Roboclaw::execute_command(u_char device_address, u_char command, u_char reply_size,
                               std::vector<u_char> &response) {
   std::vector<u_char> data;
   size_t res_size = 0;
   //std::cout << "create query" << std::endl;
   data.push_back(device_address);
   data.push_back(command);
   //std::cout << "send query" << std::endl;
   my_serial->write(data);
   data.clear();
   //std::cout << "clear query" << std::endl;
   response.clear();
   //std::cout << "read data" << std::endl;
   res_size = my_serial->read(response, reply_size);
   //std::cout << "data read" << std::endl;
   if (res_size == reply_size) {
      return true;
   } else {
      return false;
   }
}

bool Roboclaw::execute_command(std::vector<u_char> data) {
   //std::vector<u_char> response;
   my_serial->write(data);
   data.clear();
   my_serial->read(data, 1);
   if (data[0] == 0xff) {
      return true;
   } else {
      return false;
   }
}

bool Roboclaw::set_speed(double left_motor, double right_motor) {
   //std::cout << "target spd[m/s] = " << left_motor << ", " << right_motor;
   left_target_pps = (int32_t) ((left_motor) * ppm);
   right_target_pps = (int32_t) ((right_motor) * ppm);
   //std::cout << "target spd [pps]= " << left_target_pps << ", " << right_target_pps << std::endl;
   if (driveM2SignedSpeed(right_target_pps) && driveM1SignedSpeed(left_target_pps)) {
      return true;
   } else {
      return false;
   }
}

bool Roboclaw::driveM1SignedSpeed(int32_t speed) {
   clear_crc();
   crc = add_byte(crc, device_address); // address
   crc = add_byte(crc, COM_DRV_M1_SIGNED_SPD); //command Drive M1 With Signed Speed
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
   if (execute_command(l_data)) {
      //std::cout << "M1 set OK";
      return true;
   } else {
      //std::cout << "M1 set ERR";
      return false;
   }
}

bool Roboclaw::driveM2SignedSpeed(int32_t speed) {
   clear_crc();
   crc = add_byte(crc, device_address); // address
   crc = add_byte(crc, COM_DRV_M2_SIGNED_SPD); //command Drive M2 With Signed Speed
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
   if (execute_command(l_data)) {
      //std::cout << "M2 set OK";
      return true;
   } else {
      //std::cout << "M1 set ERR";
      return false;
   }
}

//TODO - impelment commands 38,39 - speed with limited acceleration

bool Roboclaw::read_version() {
   return read_version(false);
}

bool Roboclaw::read_version(bool print_version) {
   std::vector<u_char> res;
   execute_command(device_address, COM_READ_VERSION,
                   COM_READ_VERSION_REPLY_SIZE, res);
   //std::cout << "Response size: " << res.size() << std::endl;
   if (res.size() > 10) {

      std::string ver;

      //std::cout << "Received data: ";
      for (int i = 0; i < res.size(); i++) {
         //std::cout << " " << (int) res[i];
         ver.push_back((u_char) res[i]);
      }
      //std::cout << "|||" << std::endl;
      version.erase();
      version.append(ver);
      version.pop_back();
      version.pop_back();
      version.pop_back();
      if (print_version) {
         std::cout << "VERSION: " << version << std::endl;
      }
   } else {
      return false;
   }
   return true;
}

bool Roboclaw::reset_encoder_counters() {
   clear_crc();
   crc = add_byte(crc, device_address); // address
   crc = add_byte(crc, COM_RESET_ENCODERS); //command Drive M1 With Signed Speed
   add_crc();
   if (execute_command(l_data)) {
      return true;
   } else {
      return false;
   }
}

bool Roboclaw::read_encoders(long *enc1, long *enc2) {
   std::vector<u_char> response;
   //std::cout << "Execute command: read enc" << std::endl;
   if (execute_command(device_address, COM_READ_ENCODER_COUNTERS,
                       COM_READ_ENCODER_COUNTERS_REPLY_SIZE, response)) {
      //if (response.size() > 8) {
      unsigned long encoder_1 =
            ((u_char) response[0]) << 24 | ((u_char) response[1] << 16) | ((u_char)
                  response[2]) << 8 | ((u_char) response[3]);
      *enc1 = encoder_1;
      unsigned long encoder_2 =
            ((u_char) response[4]) << 24 | ((u_char) response[5] << 16) | ((u_char)
                  response[6]) << 8 | ((u_char) response[7]);
      *enc2 = encoder_2;
      //}
   } else {
      return false;
   }
   return true;
}