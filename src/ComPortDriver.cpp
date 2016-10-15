//
// Created by lukasz on 30.09.16.
//

#include "ComPortDriver.h"

ComPortDriver::ComPortDriver(int port_number) {
   port_num = port_number;
   if (init_comport(port_num, 115200)) {
      port_opened = true;
   } else {
      port_opened = false;
   }
}

ComPortDriver::~ComPortDriver() {
   close_comport(port_num);
}


int ComPortDriver::send_data(std::vector<char> data) { // theta, x, y
   int data_size = data.size();
   std::cout << "Send " << data_size << " bytes of data" << std::endl;
   for (int i = 0; i < data_size; i++) {
      std::cout << " " << (int) (u_char) data[i];
      RS232_SendByte(port_num, data[i]);
   }
   std::cout << std::endl;
   return 0;
}

std::vector<char> ComPortDriver::read_data(bool wait_for_end_of_line) {
   if (wait_for_end_of_line) {
      std::vector<char> data;
      unsigned char buffer[1];
      int data_len = 0; // length of data buffer
      int n = 0;  // number of bytes read
      int i = 0;
      bool end_of_line = false;
      //std::cout << "Reading data" << std::endl;
      n = RS232_PollComport(port_num, buffer, 1);
      if (n == -1) {
         std::cout << "Read error" << std::endl;
         return data;
      }
      while (n == 0) {
         n = RS232_PollComport(port_num, buffer, 1);
      }
      data.push_back(buffer[0]);
      while (!end_of_line) {
         n = RS232_PollComport(port_num, buffer, 1);
         if (n == 1) {
            data.push_back(buffer[0]);
            if (buffer[0] == 10) {
               end_of_line = true;
            }
         }
      }
      data_len = data.size();
      std::cout << "Read " << data_len << " bytes of data" << std::endl;
      std::cout << "Data read: ";
      //std::cout << one_sign[0] << one_sign[1] << one_sign[2] << one_sign[3] << std::endl;
      for (i = 0; i < data_len; i++) {
         std::cout << data[i];
         data.push_back(data[i]);
      }

      //std::cout << std::endl;
      return data;
   } else {
      return read_data();
   }
}

std::vector<char> ComPortDriver::read_data() {
   std::vector<char> data;

   //std::cout << "Reading data" << std::endl;
   unsigned char buffer[16];

   int n = RS232_PollComport(port_num, buffer, 16);
   //std::cout << "n=" << n << std::endl;
   std::cout << "Read " << n << " bytes of data" << std::endl;
   std::cout << "Data read: " << std::endl;
   //std::cout << one_sign[0] << one_sign[1] << one_sign[2] << one_sign[3] << std::endl;
   for (int i = 0; i < n; i++) {
      std::cout << buffer[i] << ", ";
      data.push_back(buffer[i]);
   }

   return data;
}

int ComPortDriver::init_comport(int port_number, long baud_rate) {
   char mode[] = {'8', 'N', '1', 0};
   if (RS232_OpenComport(port_number, baud_rate, mode)) {
      std::cout << "Can not open comport" << std::endl;
      return (0);
   }
   // port opened
   return 1;
}

int ComPortDriver::close_comport(int port_number) {
   RS232_CloseComport(port_number);
}