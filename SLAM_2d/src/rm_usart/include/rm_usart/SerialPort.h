//
// Created by zhiyu on 2021/8/20.
//

#ifndef AUTOAIM_SERIALPORT_H
#define AUTOAIM_SERIALPORT_H

#include <boost/asio.hpp>
#include <string>
#include <vector>
#include <queue>
#include<chrono>
#include <ros/ros.h>
#include "SerialPort.h"
#include <ros/time.h>
#include <serial/serial.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <sstream>

using namespace boost::asio;
using namespace std;
#define KALMANDEBUG
namespace ly{

    /*CRC校验*/ 
    const unsigned char CRC8_INIT = 0xff;
    const unsigned char CRC8_TAB[256] =
            {
                    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
                    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
                    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
                    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
                    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
                    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
                    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
                    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
                    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
                    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
                    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
                    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
                    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
                    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
                    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
                    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
            };



    #pragma pack(push, 1)
    struct SerialPortData{  // 顺序最好别改
        unsigned char head; //[0]
        unsigned char goal; //[1]
        unsigned char crc; //[2]
        SerialPortData(){

        }
        SerialPortData(SerialPortData *pData) {
            // 构造函数不占内存
//            memcpy(this, pData, sizeof(SerialPortData));
        }
    };

    struct SerialPortWriteData{  // 顺序最好别改
        unsigned char head; //[0]
        float x_set;			//[1][2]
        float y_set;			//[3][4]
        float w_set;			//[5][6]
        unsigned char flag; //[7]
        // unsigned char tail; //[7]
        unsigned char crc; //[8]
        SerialPortWriteData(){

        }
        SerialPortWriteData(SerialPortWriteData *pData) {
            // 构造函数不占内存
//            memcpy(this, pData, sizeof(SerialPortData));
        }
    };


    #pragma pack(pop)

    struct EularAngles{
        double roll;
        double pitch;
        double yaw;
        int lll;
    };

    struct Params_ToSerialPort{
        void* __this;
        vector<SerialPortData>* cache;
        uint8_t* id;

        Params_ToSerialPort(){
            __this = nullptr;
            cache = nullptr;
            id = new uint8_t ;
        }
        Params_ToSerialPort(vector<SerialPortData>* cache)
            : cache(cache){
            __this = nullptr;
            id = new uint8_t ;
        }
//        Params_ToSerialPort(vector<SerialPortData *> &cache, void *&_this)
//                : cache(cache), __this(_this) {
//            id = new uint8_t;
//        }
    };




    class SerialPort {
    public:
        explicit SerialPort();
        explicit SerialPort(const string& port_name);
        ~SerialPort();
        void readData(SerialPortData *imu_data);
        void writeData(SerialPortWriteData *_data_write);
        void startSerialPortRead(Params_ToSerialPort &params_to_serial_port);

    private:
        // 创建一个serial对象
        serial::Serial sp;
        // 创建timeout
        serial::Timeout to = serial::Timeout::simpleTimeout(100);

    private:
    
        serial_port* _serial_port;
        io_service _io_service;
        int _data_len = sizeof(SerialPortData); // sizeof(SerialPortData)
        int _data_len_write = sizeof(SerialPortWriteData); // sizeof(SerialPortData)
        unsigned char msg[sizeof(SerialPortWriteData)];
//        SerialPortData* _data_read;
//        SerialPortData* _data_write;
        unsigned char _data_tmp[3];
        unsigned char pingpong[3*2];
        pthread_t threadID;
        Params_ToSerialPort thread_params;
        boost::system::error_code _err;
        EularAngles angles;

        SerialPortData stm32;
        bool BeginTime_Flag = false;
        std::chrono::steady_clock::time_point time_start;
        std::chrono::steady_clock::time_point time_end;
        int MCU_BeginTime;
        queue<int> yawQueue;
        queue<short> pitchQueue;
        int yawSum=0;
        int pitchSum=0;


        void serialPortRead(uint8_t* msg, uint8_t max_len);
        void serialPortWrite(uint8_t* msg, int len);

        void addCRC(SerialPortData* msg);
        void addCRC(unsigned char* msg);

        uint8_t getCRC(SerialPortData* data);
        uint8_t getCRC(unsigned char *data);

        bool verifyCRC(SerialPortData* data);
    };
}

#endif //AUTOAIM_SERIALPORT_H