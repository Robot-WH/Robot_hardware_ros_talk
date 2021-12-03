
#ifndef _DATA_PARSE_CHAOWEI_HPP_
#define _DATA_PARSE_CHAOWEI_HPP_

#include "data_parse_interface.hpp"
#include <ros/ros.h>
    /**
     * @brief  对于数据进行解析  
     */
    class DataParseChaoWei : public DataParseInterface
    {  
        private:
            // cmd
            // 速度指令
            uint8_t speed_cmd_[9] = {0xF7, 0x81, 0x04, 0, 0, 0, 0, 0, 0}; 
            // 充电指令
            uint8_t recharge_cmd_[4] = {0xF0, 0, 0, 0}; 

        public:
           //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
           DataParseChaoWei(){};
           ~DataParseChaoWei(){};  

           //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
           /**
            * @brief 获得数据的类型  
            * @param[in] velocity  需求的线速度
            * @param[in] Angular 需求的角速度
            */
           const uint8_t* ConstructMotionCmd( const int16_t  &velocity,  const int16_t &Angular) override
           {
                // 小端字节序
                speed_cmd_[3] = velocity;  
                speed_cmd_[4] = velocity >> 8;
                speed_cmd_[5] = Angular;
                speed_cmd_[6] = Angular >> 8;  

                // CRC校验
                uint16_t crc = crc16(speed_cmd_, 7);
                speed_cmd_[7] = crc;
                speed_cmd_[8] = crc >> 8;  

                return speed_cmd_;  
           }

           //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
           /**
            * @brief 构造充电下发命令
            * @param[in]  recharge_cmd 充电命令   ， 1：对桩   0： 出桩
            */
           const uint8_t* ConstructRechargeCmd( const bool recharge_cmd) override
           {
               if(recharge_cmd)
               {
                   recharge_cmd_[1] = 0x51;  
               }
               else
               {
                   recharge_cmd_[1] = 0x52;  
               }

               // CRC校验
                uint16_t crc = crc16(recharge_cmd_, 2);
                recharge_cmd_[2] = crc;
                recharge_cmd_[3] = crc >> 8;  

                return recharge_cmd_;  
           }

           //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
           /**
            * @brief 获得数据的类型  
            * @param [in] recvBuf 数据buff
            * @param [in] start_idx 该帧数据位于当前buff的起始位置 
            */
           DataType GetDataType(const uint8_t* recvBuf) override
           {
               DataType data_type = unknow;  
               uint16_t flag_1 = recvBuf[0];
               uint16_t flag_2 = recvBuf[1];
               
               if(flag_1 == 0xA7 && flag_2 == 0x61)
               {
                   data_type = odom;
               }
               else if(flag_1 == 0xA0 && flag_2 == 0x71)
               {
               //    std::cout<<"imu flag_2: "<<flag_2<<std::endl;
                    data_type = imu;
               }
               else if(flag_1 == 0xC5 && flag_2 == 0x83)
               {
                   data_type = ultrasonic;
               }
               else if(flag_1 == 0xF0 && (flag_2 == 0x5a || flag_2 == 0x5b))
               {
                   data_type = recharge;
               }
               else if(flag_1 == 0xE1 && flag_2 == 0x63)
               {
                   data_type = stop; 
               }
               else if(flag_1 == 0xC5 && flag_2 == 0x84)
               {
                   data_type = fall;  
               }

               return data_type;  
           }

            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
           /**
            * @brief 获得数据类型的长度
            * @param [in] recvBuf 数据buff
            * @param [in] start_idx 该帧数据位于当前buff的起始位置 
            */
           uint16_t GetDataTypeLength(const DataType &data_type)
           {
               uint16_t data_length = 0;

               switch(data_type)
                {
                    // IMU
                    case imu: data_length = 23;       break;
                    // 轮速计  
                    case odom: data_length = 25;   break;  
                    // 超声波数据 
                    case ultrasonic: data_length = 21;  break;
                    // 回冲状态发布  
                    case recharge: data_length = 4; break;
                    // 急停   
                    case stop: data_length = 6; break;
                    // 防跌落
                    case fall: data_length = 7; break;
                }

                return data_length; 
           }

            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
           /**
            * @brief 读取odom数据
            */
           bool GetOdomDate(const uint8_t* recvBuf, ODOM &odom) override
           {
                // 先确定是不是odom
                uint16_t flag_1= recvBuf[0];
                uint16_t flag_2= recvBuf[1];

                if(flag_1 != 0xA7 || flag_2 != 0x61)
                {
                   return false;  
                }

                // CRC校验  
                uint16_t crc = crc16(recvBuf, 23);
                uint8_t high_crc = crc;
                uint8_t low_crc = (crc>>8);  

                if(high_crc != recvBuf[23] || low_crc != recvBuf[24])
                {
                    return false;  
                }

                // 获取数据  
                memcpy(&odom.x,  &recvBuf[3], sizeof(odom.x)); 
                memcpy(&odom.y,  &recvBuf[7], sizeof(odom.y)); 
                memcpy(&odom.theta,  &recvBuf[11], sizeof(odom.theta)); 
                memcpy(&odom.velocity,  &recvBuf[15], sizeof(odom.velocity)); 
                memcpy(&odom.angular,  &recvBuf[19], sizeof(odom.angular));
                //std::cout<<"odom theta: "<<odom.theta<<std::endl;
                // 单位换算   mm->m
                odom.x /= 1000;
                odom.y /= 1000;
                odom.velocity /= 1000;
                return true;  
           }

           //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
           /**
            * @brief 读取IMU数据
            */
           bool GetImuDate(const uint8_t* recvBuf, IMU &imu) override
           {
               // 先判断是不是odom
                uint16_t flag_1= recvBuf[0];
                uint16_t flag_2= recvBuf[1];

                if(flag_1 != 0xA0 || flag_2 != 0x71)
                {
                    return false;  
                }

                // CRC校验  
                uint16_t crc = crc16(recvBuf, 21);
                uint8_t high_crc = crc;
                uint8_t low_crc = (crc>>8);  

                if(high_crc != recvBuf[21] || low_crc != recvBuf[22])
                {
                    return false;  
                }
               // 获取数据  
               short int read_pitch, read_roll, read_yaw; 
               memcpy(&read_pitch,  &recvBuf[3], sizeof(read_pitch)); 
               memcpy(&read_roll,  &recvBuf[5], sizeof(read_roll)); 
               memcpy(&read_yaw,  &recvBuf[7], sizeof(read_yaw)); 
               short int read_gyro_x, read_gyro_y, read_gyro_z; 
               memcpy(&read_gyro_x,  &recvBuf[9], sizeof(read_gyro_x)); 
               memcpy(&read_gyro_y,  &recvBuf[11], sizeof(read_gyro_y)); 
               memcpy(&read_gyro_z,  &recvBuf[13], sizeof(read_gyro_z)); 
               short int  read_accel_x, read_accel_y, read_accel_z; 
               memcpy(&read_accel_x,  &recvBuf[15], sizeof(read_accel_x)); 
               memcpy(&read_accel_y,  &recvBuf[17], sizeof(read_accel_y)); 
               memcpy(&read_accel_z,  &recvBuf[19], sizeof(read_accel_z)); 
               imu.pitch = read_pitch*0.1 * CoefDegreeToRadian;
               imu.roll = read_roll*0.1 * CoefDegreeToRadian;
               imu.yaw = read_yaw*0.1 * CoefDegreeToRadian;
               //std::cout << "pry " << read_pitch << " " << read_roll<< " " << read_yaw<< std::endl;
               imu.gyro[0] = read_gyro_x*0.1 * CoefDegreeToRadian;
               imu.gyro[1] = read_gyro_y*0.1 * CoefDegreeToRadian;
               imu.gyro[2] = read_gyro_z*0.1 * CoefDegreeToRadian;
               imu.accel[0] = read_accel_x*0.009806;
               imu.accel[1] = read_accel_y*0.009806;
               imu.accel[2] = read_accel_z*0.009806;

               return true;  
           }

           //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
           /**
            * @brief 读取超声波数据
            */
           bool GetUltrasonicData(const uint8_t* recvBuf, std::vector<uint16_t> &datas) override
           {
               // 先判断是不是odom
                uint16_t flag_1= recvBuf[0];
                uint16_t flag_2= recvBuf[1];

                if(flag_1 != 0xC5 || flag_2 != 0x83)
                {
                   return false;  
                }
                // CRC校验  
                uint16_t crc = crc16(recvBuf, 19);
                uint8_t high_crc = crc;
                uint8_t low_crc = (crc>>8);  

                if(high_crc != recvBuf[19] || low_crc != recvBuf[20])
                {
                    return false;  
                }
               
               if(!datas.empty())
               {
                    datas.clear();  
               }

               // 读取超声的个数
               uint8_t  ultrasonic_num = recvBuf[2] / 2;
 
               for(uint8_t i = 0; i<ultrasonic_num; i++)
               {
                   uint16_t distance;
                   memcpy(&distance,  &recvBuf[3+i*2], sizeof(distance)); 
                   distance = distance / 10;
                   datas.push_back(std::move(distance)); 
               }

               return true;  
           }

           //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
           /**
            * @brief 读取充电状态
            */
           bool GetRechargeStatus(const uint8_t* recvBuf, int8_t &status) override
           {
               // 先判断是不是充电状态
                uint16_t flag= recvBuf[0];

               if(flag != 0xF0)
               {
                   return false;  
               } 
               
                status = recvBuf[1];
                return true;  
           }

            //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /**
            * @brief 读取急停状态
            */
           bool GetStopStatus(const uint8_t* recvBuf, int8_t &status) override
           {
               // 先判断是不是充电状态
                uint16_t flag_1= recvBuf[0];
                uint16_t flag_2= recvBuf[1];

               if(flag_1 != 0xE1 || flag_2 != 0x63)
               {
                   return false;  
               } 
               
                status = recvBuf[3];
                return true;  
           }

    };  

#endif 
