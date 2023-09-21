//
// Created by zhiyu on 2021/8/20.
//

#include "SerialPort.h"
#include <cstring>
#include <iostream>
//#define KALMANDEBUG
namespace ly
{

    SerialPort::SerialPort()
    {
        // string _id;
        // string save_path = "/dev/ttyUSB*";
        // FILE *fp = popen(("ls " + save_path).c_str(), "r");
        // std::fscanf(fp, "%s", _id.begin());
        // pclose(fp);
        // _id.pop_back();
        // ROS_INFO("%s", _id.writeDatac_str());
        // new(this) SerialPort(/*SerialPortParam::deviceName*/_id);
    }

    SerialPort::SerialPort(const string &port_name)
    {
    
        try
        {
            // 设置要打开的串口名称
            sp.setPort(port_name);
            // 设置串口通信的波特率
            sp.setBaudrate(115200);
            // 串口设置timeout
            sp.setTimeout(to);

            // 打开串口
            sp.open();
        
            // 判断串口是否打开成功
            if (sp.isOpen())
            {
                ROS_INFO_STREAM("/dev/ttyUSB* is opened.");
            }

            // this->_serial_port = new serial_port(_io_service, port_name);
            // this->_serial_port->set_option(serial_port::baud_rate(115200));
            // this->_serial_port->set_option(serial_port::flow_control(serial_port::flow_control::none));
            // this->_serial_port->set_option(serial_port::parity(serial_port::parity::none));
            // this->_serial_port->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
            // this->_serial_port->set_option(serial_port::character_size(8));

            // _data_tmp = (uint8_t *)malloc((size_t)_data_len* 2);
            // pingpong = (uint8_t *)malloc((size_t)_data_len * 2+1);
        }
        catch (serial::IOException &e)
        {
                ROS_ERROR_STREAM("Unable to open port.");
        }
        catch (...)
        {
            ROS_ERROR( "create serial port object error! ");
        }
    }

    SerialPort::~SerialPort()
    {
        sp.close();
        free(_data_tmp);
        free(pingpong);
        delete _serial_port;
    }

    void SerialPort::serialPortRead(uint8_t *msg, uint8_t max_len)
    {
        try
        {
            read(*_serial_port, boost::asio::buffer(msg, max_len), _err);
        }
        catch (...)
        {
            int aaa=11;
            ROS_ERROR("readData from serial port error! ");
//            string _id;
//            string save_path = "/dev/ttyUSB*";
//            FILE *fp = popen(("ls " + save_path).c_str(), "r");
//            std::fscanf(fp, "%s", _id.begin());
//            pclose(fp);
//            _id.pop_back();
//            new(this) SerialPort(/*SerialPortParam::deviceName*/_id);
            exit(0);
        }
    }

    void SerialPort::serialPortWrite(uint8_t *msg, int len)
    {
        try
        {
            write(*_serial_port, buffer(msg, (size_t)len));
        }
        catch (...)
        {
            ROS_ERROR("write to serial port error! ");
//            string _id;
//            string save_path = "/dev/ttyUSB*";
//            FILE *fp = popen(("ls " + save_path).c_str(), "r");
//            std::fscanf(fp, "%s", _id.begin());
//            pclose(fp);
//            _id.pop_back();
//            new(this) SerialPort(/*SerialPortParam::deviceName*/_id);
            exit(0);
        }
    }

    void SerialPort::addCRC(SerialPortData *msg)
    {
        if (!msg)
            return;
        msg->crc = getCRC(msg);
    }

    void SerialPort::addCRC(unsigned char *msg)
    {
        if (!msg)
            return;
        msg[_data_len_write - 1] = getCRC(msg);
    }

    uint8_t SerialPort::getCRC(SerialPortData *data)
    {
        auto _data = reinterpret_cast<unsigned char *>(data);
        int dwLength = _data_len - 1;
        unsigned char ucCRC8 = CRC8_INIT;
        unsigned char ucIndex;
        while (dwLength--)
        {
            ucIndex = ucCRC8 ^ (*_data++);
            ucCRC8 = CRC8_TAB[ucIndex];
        }

        return ucCRC8;
    }

    uint8_t SerialPort::getCRC(unsigned char *data)
    {
        auto _data = reinterpret_cast<unsigned char *>(data);
        int dwLength = _data_len_write - 1;
        unsigned char ucCRC8 = CRC8_INIT;
        unsigned char ucIndex;
        while (dwLength--)
        {
            ucIndex = ucCRC8 ^ (*_data++);
            ucCRC8 = CRC8_TAB[ucIndex];
        }

        return ucCRC8;
    }

    bool SerialPort::verifyCRC(SerialPortData *data)
    {
        if (!data)
            return false;
        return data->crc == getCRC(data);
    }

    void SerialPort::readData(SerialPortData *msg_data)
    {
        //    auto data_read = reinterpret_cast<SerialPortData*>(params_p);
        // serialPortRead(_data_tmp, _data_len);
        // Receive data
        unsigned char _data_tmp[sizeof(SerialPortData)];
        //unsigned char pingpong[sizeof(SerialPortData)*2];

        int n = sp.read(_data_tmp, _data_len);
                    //ROS_DEBUG("test0:%d ",_data_tmp[0]);
                    //ROS_DEBUG("test1:%d ",_data_tmp[1]);
                    //ROS_DEBUG("test2:%d \n",_data_tmp[2]);
                    
        //ROS_DEBUG("Read n:%d\n",n);
        if(n==_data_len){
            memcpy(pingpong, _data_tmp, _data_len);
            memcpy(pingpong + _data_len, _data_tmp, _data_len);
            for (int start_bit = 0; start_bit < _data_len; start_bit++)
            {
                if (pingpong[start_bit] == '!')
                {
                    //ROS_DEBUG("test0:%d ",pingpong[start_bit]);
                    //ROS_DEBUG("test1:%d ",pingpong[start_bit+1]);
                    //ROS_DEBUG("test2:%d \n",pingpong[start_bit+2]);
                    
                    memcpy(_data_tmp, pingpong + start_bit, _data_len);
                     
                    //cout<<"func:"<<pingpong[start_bit+1]<<endl;
                    //@TODO CRC校验
                    //if (verifyCRC(reinterpret_cast<SerialPortData *>(_data_tmp))) //CRC校验
                    if (1)
                    {
                        memcpy((unsigned char *)msg_data, _data_tmp, _data_len);
                        //ROS_DEBUG("Read x_now:%d \tYAW: %.3f \tw_now: %f",msg_data->x_now,msg_data->Yaw_now,msg_data->w_now);
                        break;
                    }
                }
            }
            
            memcpy(pingpong, pingpong + _data_len, _data_len);
        }
        
    }

    void SerialPort::writeData(SerialPortWriteData *_data_write)
    {
        memcpy(msg,_data_write,sizeof(SerialPortWriteData));
//        cout<<"wsize:"<<sizeof(SerialPortWriteData)<<endl;

//        std::cout<<"send x:"<<(int)_data_write->x<<" y:"<<(int)_data_write->y<<"\n";
        addCRC(msg);
//        for(int i=0;i<sizeof(msg);i++) printf("%x ",msg[i]);
//        cout<<endl;
        // serialPortWrite(msg, _data_len_write);
        sp.write(msg, _data_len_write); 
    }

}
