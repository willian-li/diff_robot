#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class Stm32Comms
{

public:

  Stm32Comms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    serial_conn_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_conn_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  void send_msg(const std::vector<unsigned char> &msg_to_send)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);
  }

  void getInfo()
  {
    std::vector<unsigned char> outputData = {0x5a, 0x06, 0x01, 0x13, 0x00, 0x33};
    send_msg(outputData);
  }

  void read_data(double &Vx,double &Vyaw)
  {
    std::vector<unsigned char> data_vector;

    serial_conn_.Read(data_vector, 12);
    if(data_vector[3] == 0x12)
    {
      int16_t v = (static_cast<int16_t>(data_vector[4]) << 8) | static_cast<int16_t>(data_vector[5]);
      Vx = static_cast<double>(v)/1000;
      
      int16_t yaw = (static_cast<int16_t>(data_vector[10]) << 8) | static_cast<int16_t>(data_vector[11]);
      Vyaw = static_cast<double>(yaw)/1000;
    }

    // // 打印接收数据
    // for (unsigned char byte : data_vector) 
    // {
    //   std::cout << static_cast<double>(byte) << " ";
    // }
    // std::cout << std::endl;
  }
  void get_odom(double &Vx,double &Vyaw)
  {
    std::vector<unsigned char> outputData = {0x5a, 0x06, 0x01, 0x11, 0x00, 0xa2};
    send_msg(outputData);
    read_data(Vx,Vyaw);
  }
  unsigned char crc_1byte(unsigned char data) {
    unsigned char crc_1byte = 0;
    for (int i = 0; i < 8; i++) {
        if ((crc_1byte ^ data) & 0x01) {
            crc_1byte ^= 0x18;
            crc_1byte >>= 1;
            crc_1byte |= 0x80;
        } else {
            crc_1byte >>= 1;
        }
        data >>= 1;
    }
    return crc_1byte;
  }

  unsigned char crc_byte(std::vector<unsigned char> data, int length) {
      unsigned char ret = 0;
      for (int i = 0; i < length; i++) {
          ret = crc_1byte(ret ^ data[i]);
      }
      return ret;
  }

  void write_cmd(const double &linear,const double &angular)
  {
    std::vector<unsigned char> outputData = {0x5a,0x0c,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    outputData[4] = (static_cast<int>(linear * 1000.0) >> 8) & 0xff;
    outputData[5] = static_cast<int>(linear*1000.0)&0xff;
    outputData[6] = (static_cast<int>(0*1000.0)>>8)&0xff;
    outputData[7] = static_cast<int>(0*1000.0)&0xff;
    outputData[8] = (static_cast<int>(angular*1000.0)>>8)&0xff;
    outputData[9] = static_cast<int>(angular*1000.0)&0xff;
    unsigned char crc_8 = crc_byte(outputData, outputData.size() - 1);
    outputData[11] = crc_8;
    send_msg(outputData);
  }

  void get_imu(std::vector<double> &angular_velocity,std::vector<double> &linear_acceleration,std::vector<double> &orientation)
  {
    //发送请求获取imu
    std::vector<unsigned char> outputData = {0x5a, 0x06, 0x01, 0x13, 0x00, 0x33};
    send_msg(outputData);
    //读取数据
    std::vector<unsigned char> data_vector;
    serial_conn_.Read(data_vector, 36);
    //处理数据
    if(data_vector[3] == 0x14)
    {
      std::vector<int> int_vector;
      for(auto it = data_vector.begin(); it != data_vector.end(); ++it)
      {
          int value = static_cast<int>(*it);
          int_vector.push_back(value); 
      }
      angular_velocity[0] = static_cast<double>(static_cast<int32_t>(((int_vector[4]&0xff)<<24)|((int_vector[5]&0xff)<<16)|((int_vector[6]&0xff)<<8)|(int_vector[7]&0xff)))/100000;
      angular_velocity[1] = static_cast<double>(static_cast<int32_t>(((int_vector[8]&0xff)<<24)|((int_vector[9]&0xff)<<16)|((int_vector[10]&0xff)<<8)|(int_vector[11]&0xff)))/100000;
      angular_velocity[2] = static_cast<double>(static_cast<int32_t>(((int_vector[12]&0xff)<<24)|((int_vector[13]&0xff)<<16)|((int_vector[14]&0xff)<<8)|(int_vector[15]&0xff)))/100000;
      
      linear_acceleration[0] = static_cast<double>(static_cast<int32_t>(((int_vector[16]&0xff)<<24)|((int_vector[17]&0xff)<<16)|((int_vector[18]&0xff)<<8)|(int_vector[19]&0xff)))/100000;
      linear_acceleration[1] = static_cast<double>(static_cast<int32_t>(((int_vector[20]&0xff)<<24)|((int_vector[21]&0xff)<<16)|((int_vector[22]&0xff)<<8)|(int_vector[23]&0xff)))/100000;
      linear_acceleration[2] = static_cast<double>(static_cast<int32_t>(((int_vector[24]&0xff)<<24)|((int_vector[25]&0xff)<<16)|((int_vector[26]&0xff)<<8)|(int_vector[27]&0xff)))/100000;
      
      orientation[0] = static_cast<double>(static_cast<int16_t>((int_vector[28]&0xff)<<8|(int_vector[29]&0xff)))/10000;
      orientation[1] = static_cast<double>(static_cast<int16_t>((int_vector[30]&0xff)<<8|(int_vector[31]&0xff)))/10000;
      orientation[2] = static_cast<double>(static_cast<int16_t>((int_vector[32]&0xff)<<8|(int_vector[33]&0xff)))/10000;
      orientation[3] = static_cast<double>(static_cast<int16_t>((int_vector[34]&0xff)<<8|(int_vector[35]&0xff)))/10000;
    }
  }


private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP