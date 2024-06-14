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

  std::string send_msg(const std::vector<unsigned char> &msg_to_send)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    // try
    // {
    //   // Responses end with \r\n so we will read up to (and including) the \n.
    //   serial_conn_.ReadLine(response, '\n', timeout_ms_);
    // }
    // catch (const LibSerial::ReadTimeout&)
    // {
    //     std::cerr << "The ReadByte() call has timed out." << std::endl ;
    // }

    // if (print_output)
    // {
    //   std::cout << " Recv: " << response << std::endl;
    // }

    return response;
  }

  void getInfo()
  {
    std::vector<unsigned char> outputData = {0x5a, 0x06, 0x01, 0x13, 0x00, 0x33};
    std::string response = send_msg(outputData);
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
    std::string response = send_msg(outputData);
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
    std::string response = send_msg(outputData);
  }

  // void deal_data(const std::vector<unsigned char>& data_vector)
  // {
  //   if(data_vector[3] == 0x04)
  //   {
  //     Vx =    data_vector[4]*256;
  //     Vx +=   data_vector[5];
  //     Vy =    data_vector[6]*256;
  //     Vy +=   data_vector[7];
  //     Vyaw =  data_vector[8]*256;
  //     Vyaw += data_vector[9];
  //   }
  //   else if(data_vector[3] == 0x06)
  //   {
  //     Yawz =  data_vector[8]*256;
  //     Yawz += data_vector[9];
  //   }
  //   else if(data_vector[3] == 0x08)
  //   {
  //     Vvoltage = data_vector[4]*256;
  //     Vvoltage += data_vector[5];
  //     Icurrent = data_vector[6]*256;
  //     Icurrent += data_vector[7];
  //   }
  //   else if (data_vector[3] == 0x0a)
  //   {
  //     Vx =    data_vector[4]*256;
  //     Vx +=   data_vector[5];
  //     Yawz =  data_vector[6]*256;
  //     Yawz += data_vector[7];
  //     Vyaw =  data_vector[8]*256;
  //     Vyaw += data_vector[9];
  //   }
  //   else if(data_vector[3] == 0x12)
  //   {
  //     Vx =    data_vector[4]*256;
  //     Vx +=   data_vector[5];
  //     Vy =  data_vector[6]*256;
  //     Vy += data_vector[7];
  //     Yawz =  data_vector[8]*256;
  //     Yawz += data_vector[9];    
  //     Vyaw =  data_vector[10]*256;
  //     Vyaw += data_vector[11]; 
  //   }
  //   else if(data_vector[3] == 0x14)
  //   {
  //     Gyro[0] = int(((data_vector[4]&0xff)<<24)|((data_vector[5]&0xff)<<16)|((data_vector[6]&0xff)<<8)|(data_vector[7]&0xff));
  //     Gyro[1] = int(((data_vector[8]&0xff)<<24)|((data_vector[9]&0xff)<<16)|((data_vector[10]&0xff)<<8)|(data_vector[11]&0xff));
  //     Gyro[2] = int(((data_vector[12]&0xff)<<24)|((data_vector[13]&0xff)<<16)|((data_vector[14]&0xff)<<8)|(data_vector[15]&0xff));

  //     Accel[0] = int(((data_vector[16]&0xff)<<24)|((data_vector[17]&0xff)<<16)|((data_vector[18]&0xff)<<8)|(data_vector[19]&0xff));
  //     Accel[1] = int(((data_vector[20]&0xff)<<24)|((data_vector[21]&0xff)<<16)|((data_vector[22]&0xff)<<8)|(data_vector[23]&0xff));
  //     Accel[2] = int(((data_vector[24]&0xff)<<24)|((data_vector[25]&0xff)<<16)|((data_vector[26]&0xff)<<8)|(data_vector[27]&0xff));

  //     Quat[0] = int((data_vector[28]&0xff)<<8|data_vector[29]);
  //     Quat[1] = int((data_vector[30]&0xff)<<8|data_vector[31]);
  //     Quat[2] = int((data_vector[32]&0xff)<<8|data_vector[33]);
  //     Quat[3] = int((data_vector[34]&0xff)<<8|data_vector[35]);
  //   }
  //   else if(data_vector[3] == 0x1a)
  //   {
  //     Sonar[0] = data_vector[4];
  //     Sonar[1] = data_vector[5];
  //     Sonar[2] = data_vector[6];
  //     Sonar[3] = data_vector[7];    
  //   }
  //   else if(data_vector[3] == 0xf2)
  //   {
  //     movebase_hardware_version[0] = data_vector[4];
  //     movebase_hardware_version[1] = data_vector[5];
  //     movebase_hardware_version[2] = data_vector[6];
  //     movebase_firmware_version[0] = data_vector[7];
  //     movebase_firmware_version[1] = data_vector[8];
  //     movebase_firmware_version[2] = data_vector[9];
    
  //   }



  // }


private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
    // double pose_x = 0.0;
    // double pose_y = 0.0;
    // double pose_yaw = 0.0;
    // double serialIDLE_flag = 0;
    // double trans_x = 0.0;
    // double trans_y = 0.0;
    // double rotat_z = 0.0;
    // double speed = 0.0;
    // double steering_angle = 0.0;
    // double sendcounter = 0;
    // bool ImuErrFlag = false;
    // bool EncoderFlag = false;
    // bool BatteryFlag = false;
    // double OdomTimeCounter = 0;
    // double BatteryTimeCounter = 0;
    // double Vx = 0;
    // double Vy = 0;
    // double Vyaw = 0;
    // double Yawz = 0;
    // double Vvoltage = 0;
    // double Icurrent = 0;
    // std::vector<double> Gyro = {0,0,0};
    // std::vector<double> Accel = {0,0,0};
    // std::vector<double> Quat = {0,0,0,0};
    // std::vector<double> Sonar = {0,0,0,0};
    // std::vector<double> movebase_firmware_version = {0,0,0};
    // std::vector<double> movebase_hardware_version = {0,0,0};
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP