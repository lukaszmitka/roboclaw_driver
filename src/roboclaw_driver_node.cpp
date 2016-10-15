//
// Created by lukasz on 14.10.16.
//

//
// Created by lukasz on 30.09.16.
//

#include "roboclaw_driver_node.h"

int port_num = 16;
//bool verbose = false;

//Calculates CRC16 of nBytes of data in byte array message
unsigned int crc16(unsigned char *packet, int nBytes) {
   unsigned int crc = 0;
   for (int byte = 0; byte < nBytes; byte++) {
      crc = crc ^ ((unsigned int)packet[byte] << 8);
      for (unsigned char bit = 0; bit < 8; bit++) {
         if (crc & 0x8000) {
            crc = (crc << 1) ^ 0x1021;
         } else {
            crc = crc << 1;
         }
      }
   }
   return crc;
}

int main(int argc, char **argv) {

   ros::init(argc, argv, "herkulex_servo_driver");

   ros::NodeHandle n("roboclaw_driver");



   ComPortDriver comPortDriver(port_num);
   std::vector<char> data;

   //std::cout << "Send reboot signal:" << std::endl;


   data.push_back(0x80);
   data.push_back(21);
   comPortDriver.send_data(data);
   comPortDriver.read_data(true);
   /*for (int i=0; i<100; i++){
      comPortDriver.read_data(true);
      ros::Duration(0.01).sleep();
   }*/

   //ros::Duration(2.5).sleep();

   /*std::cout << "Send EEP_READ signal:" << std::endl;
   data.clear();
   data.push_back(0xFF);
   data.push_back(0xFF);
   packet_size = 9;
   data.push_back(packet_size);
   servo_id = 219;
   data.push_back(servo_id);
   command = 0x02;
   data.push_back(command);
   char data_0 = 6;
   char data_1 = 1;
   //char data_2 = 0xDB;
   checksum_1 = (packet_size ^ servo_id ^ command ^ data_0 ^ data_1) & 0xFE;
   checksum_2 = (~(packet_size ^ servo_id ^ command ^ data_0 ^ data_1)) & 0xFE;
   std::cout << "CS1: " << (int) (uint8_t) checksum_1 << ", CS2: " << (int) (uint8_t) checksum_2 <<
   std::endl;
   data.push_back(checksum_1);
   data.push_back(checksum_2);
   data.push_back(data_0);
   data.push_back(data_1);
   //data.push_back(data_2);
   comPortDriver.send_data(data);
   comPortDriver.read_data();
   ros::Duration(2.5).sleep();

   std::cout << "Send reboot signal:" << std::endl;
   data.clear();
   data.push_back(0xFF);
   data.push_back(0xFF);
   packet_size = 0x07;
   data.push_back(packet_size);
   servo_id = 0xFE;
   data.push_back(servo_id);
   command = 0x09;
   data.push_back(command);
   checksum_1 = (packet_size ^ servo_id ^ command) & 0xFE;
   checksum_2 = (~(packet_size ^ servo_id ^ command)) & 0xFE;
   std::cout << "CS1: " << (int) (uint8_t) checksum_1 << ", CS2: " << (int) (uint8_t) checksum_2 <<
   std::endl;
   data.push_back(checksum_1);
   data.push_back(checksum_2);
   comPortDriver.send_data(data);
   comPortDriver.read_data();*/

   return 0;
}


