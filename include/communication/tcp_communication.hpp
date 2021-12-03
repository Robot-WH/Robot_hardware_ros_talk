#ifndef _TCP_COMMUMICATION_HPP_
#define _TCP_COMMUMICATION_HPP_

#include<unistd.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<stdio.h>
#include<stdlib.h>
#include<fcntl.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<sys/ioctl.h>
#include<string.h>

/**
 * @brief  TCP 通信实现信息的读取与发送  
 */
class TcpCommunicator
{
    private:
            struct sockaddr_in addrSrc_;
            int clientSocket_ = -1;  
            bool is_connect_ = false; 
                
    public:
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        TcpCommunicator(){};
        ~TcpCommunicator(){};
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool GetConnectStatus() const
        {
            return is_connect_;  
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        bool SetConnectOff() 
        {
            close(clientSocket_);      // 主动断开连接 
            is_connect_ = false;  
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
        * @brief  建立一次连接
        */
        bool Connect(const int &port_num, const char* net_addr)
        {
            //创建一个socket
            clientSocket_=socket(AF_INET, SOCK_STREAM, 0);    // SOCK_STREAM（流格式套接字/面向连接的套接字)  即 TCP   ，  SOCK_DGRAM 为UDP

            if(clientSocket_ == -1)  
            {
                return false;  
            }

            //socket设置为非阻塞 
            unsigned long on = 1;
            if (ioctl(clientSocket_, FIONBIO, &on) < 0) 
            {
                //cout<<"ioctlsocket failed"<<endl;
                return false;
            }

            memset(&addrSrc_, 0, sizeof(struct sockaddr_in));
            addrSrc_.sin_family=AF_INET;
            addrSrc_.sin_port=htons(port_num);    
            addrSrc_.sin_addr.s_addr=inet_addr(net_addr);
            timeval tm;
            fd_set set;
            int error=-1, len;
            bool ret = false;
            //  LOG(INFO)<<"tcp connecting ......."; 
            if(connect(clientSocket_, (const struct sockaddr *)&addrSrc_, sizeof(struct sockaddr_in)) == -1)    // 如果不连接 会一直阻塞  直到超时
            {
                tm.tv_sec = 1;     //  超时时间   /秒    ,   即在select中超时等待的时间   ，这个时间内没完成连接则失败  
                tm.tv_usec = 0;
                FD_ZERO(&set);
                FD_SET(clientSocket_, &set);

                if(select(clientSocket_+1, NULL, &set, NULL, &tm) > 0)
                {
                    getsockopt(clientSocket_, SOL_SOCKET, SO_ERROR, &error, (socklen_t *)&len);

                    if(error == 0) 
                    {
                        ret = true;
                    }
                    else 
                    {
                        ret = false;
                    }
                } 
                else 
                {
                    ret = false;
                }
            }
            else 
            {
                ret = true;
            }

            if(!ret)
            {
                close( clientSocket_ );
                std::cout<<"connect false! "<<std::endl;
                return false;
            }

            is_connect_ = true;  
            std::cout<<"connect success!!! "<<std::endl;
            return  true;  
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
        * @brief 一次数据读取  
        * @param[in]  received_size 接收buff区大小
        * @param[out]  received_num 本次接收数量  
        */
        bool DataReceived(uint8_t* recvBuf, const int &received_size, int16_t &received_num)
        {
            received_num = -1;   
            // 接受信息    
            int16_t received_num_ =  recv(clientSocket_, recvBuf, received_size, 0);

            if(received_num_ > 0)
            {
                received_num = received_num_;  
                // std::cout<<"received num: "<<received_num<<std::endl;
                return true;
            }
            
            return false;  
        }
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
        * @brief 一次数据发送
        * @param[in] dataBuf 要发送的数据 
        */
        void DataSend(const uint8_t* dataBuf, const uint16_t &send_num)
        {
            // 检查是否连接了
            if(!is_connect_)  
            {
                return;
            } 
            // 发送
            send(clientSocket_, dataBuf, send_num, 0);
        }
                                       
};  

#endif  