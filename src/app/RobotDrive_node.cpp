/*********************************************************************************************
 * @brief ROS-机器人底层通信接口
 * @author lwh
 * @date  2021./7/15
 *********************************************************************************************/
#include "ros_utils.hpp"
#include "utility.hpp"
#include "communication/tcp_communication.hpp"
#include "robot_drive/RechargeStatus.h"
#include "robot_drive/RechargeStatusRequest.h"
#include "robot_drive/RechargeStatusResponse.h"
#include "robot_drive/StopStatus.h"
#include "robot_drive/StopStatusRequest.h"
#include "robot_drive/StopStatusResponse.h"
#include "data_parse_chaowei.hpp"
//#include "ubt_key.h"

using namespace std;  

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 typedef enum
 {
    RECHARGE_UNKNOWN,
    RECHARGE_START_RUNNING,
    RECHARGE_FINISH_RUNNING
 }RechargeStatusType;

 typedef enum
 {
    STOP_IDLE,
    STOP_SOFT,
    STOP_HARD
 }StopStatusType;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class RobotDriveNode
{
     private:
        ros::NodeHandle nh_;  
        //  TCP 通信接口    
        TcpCommunicator tcp_communicator_;
        //  数据解析器  
        std::unique_ptr<DataParseInterface> data_parse_ptr_;  
        int receive_size_ = 0; 
        // Topic 
        ros::Publisher imu_pub_;
        ros::Publisher odom_pub_;
        ros::Publisher ultrasonic_pub_;
        ros::Publisher recharge_pub_;        // 回充话题
        ros::Publisher stop_pub_;                // 急停话题
        ros::Subscriber speed_sub_;  
        ros::Subscriber recharge_sub_;        // 回充话题
        ros::ServiceServer get_recharge_status_server_;
        ros::ServiceServer get_stop_status_server_;
        //定义发布器对象
	    tf::TransformBroadcaster  odom_broadcaster_;//TF发布器
        // geometry_msgs::TransformStamped odom_trans;
        RechargeStatusType recharge_status_; //回充状态
        StopStatusType stop_status_;
        // 通信信息 
        const int port_num_;
        const char *net_addr_;
        // 连接判断计数
        uint16_t connect_count_ = 0;  
        // 互斥锁
        std::mutex  connect_mutex_;  
        // 数据缓存
        std::queue<uint8_t *> received_buff_;   
        std::queue<int16_t> received_num_container_;   
        
    public:
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        RobotDriveNode(const int& port_num, const char* net_addr, const int &receive_size, std::unique_ptr<DataParseInterface> data_parse_ptr) 
                                            : receive_size_(receive_size) , port_num_(port_num) ,net_addr_(net_addr)  ,data_parse_ptr_(std::move(data_parse_ptr))
        {  
            // imu topic  发布
            imu_pub_ = nh_.advertise<sensor_msgs::Imu>( "/imu_raw_data", 1000 );
            // odom topic 发布  
            odom_pub_ = nh_.advertise<nav_msgs::Odometry>( "/odom", 1000 );
            // 超声话题发布  
            ultrasonic_pub_ = nh_.advertise<cruiser_msgs::cruiserSensorAltrasonic>( "/sensor_untrasonic", 1000 );
            recharge_pub_ = nh_.advertise<std_msgs::Int8>( "/recharge_feedback", 10);
            stop_pub_ = nh_.advertise<std_msgs::Int8>( "/stop_status", 10);
            // 速度信息接收
            speed_sub_ = nh_.subscribe("/cmd_vel_chassis", 1000,  &RobotDriveNode::motionHandler, this, ros::TransportHints().tcpNoDelay());
            //  回充
            recharge_sub_ = nh_.subscribe("/cmd_recharge", 1000,  &RobotDriveNode::rechargeHandler, this, ros::TransportHints().tcpNoDelay());
            recharge_status_ = RECHARGE_UNKNOWN; //回充unknkown状态
            get_recharge_status_server_ = nh_.advertiseService("get_recharge_status", &RobotDriveNode::getRechargeStatusService, this);
            stop_status_ = STOP_IDLE; //回充unknkown状态
            get_stop_status_server_ = nh_.advertiseService("get_stop_status", &RobotDriveNode::getStopStatusService, this);
            // 进行通信连接   
            if(tcp_communicator_ .Connect(port_num_, net_addr_))
            {
                ROS_INFO("connect OK!!! " );
            }
            else
            {
                ROS_INFO("cconnect failure!!!" );
            }
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 数据接收处理线程 
         */ 
        void DataRecived()
        {
            while(true)
            {   
                if(tcp_communicator_.GetConnectStatus())
                {
                    uint8_t *recvBuf = new uint8_t[receive_size_];
                    int16_t received_num = 0;  
                   // 进行一次接收 
                   if(tcp_communicator_.DataReceived(recvBuf, receive_size_, received_num))
                   {
                        std::lock_guard<std::mutex> lock(connect_mutex_);
                        connect_count_ = 0;
                        received_buff_.push(recvBuf); 
                        received_num_container_.push(received_num);  
                   }
                   else
                   {
                       delete [] recvBuf;  
                   }
                }
                // 1us  延时     microseconds    milliseconds
                std::chrono::microseconds dura(1);
                std::this_thread::sleep_for(dura);
            }
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 数据接收处理线程 
         */ 
        void DataParse()
        {
            while(true)
            {   
                if (received_buff_.empty() || received_num_container_.empty())
                {
                    continue;
                }

                uint8_t *recvData = received_buff_.front();
                int16_t recvNum = received_num_container_.front();
                received_num_container_.pop();   
                received_buff_.pop();   
                uint16_t start_idx = 0;   

                while(start_idx <= recvNum - 1) 
                {
                    // 解析  
                    DataType data_type = data_parse_ptr_->GetDataType(&recvData[0] + start_idx);      // step1: 首先判断是什么类型数据
                    if(data_type == unknow) {
                        break;
                    }
                    // 获取数据类型的长度
                    uint16_t data_length = data_parse_ptr_->GetDataTypeLength(data_type);
                    if(recvNum  - start_idx  < data_length)
                    {
                        break;
                    }
                    // step2: 根据数据类型  解析并且发布
                    switch(data_type)
                    {
                        case imu:
                        {
                                IMU imu_data;  
                                if( !data_parse_ptr_->GetImuDate(&recvData[0] + start_idx,  imu_data) )  
                                {
                                    break;
                                }
                                // 将IMU数据发布出去 
                                publicImuTopic(imu_data);
                        }
                        break;
                        case odom:
                        {
                                ODOM odom_data;  
                                if( !data_parse_ptr_->GetOdomDate(&recvData[0] + start_idx,  odom_data) )  
                                {
                                    break;
                                }
                                // 计算速度   
                                ros::Time now_stamp = ros::Time::now();
                                // 将odom数据发布出去  
                                publishOdomTopic(odom_pub_, odom_broadcaster_, odom_data.x, odom_data.y, odom_data.theta * CoefDegreeToRadian, 
                                                                        odom_data.velocity, odom_data.angular, ros::Time::now(), "odom", "base_footprint");
                        }
                        break;  
                        // 超声波数据 
                        case ultrasonic:
                        {
                                std::vector<uint16_t> distance_values; 
                                if(!data_parse_ptr_->GetUltrasonicData(&recvData[0] + start_idx, distance_values))
                                {
                                    break;
                                }  
                                publishUltrasonicTopic(ultrasonic_pub_,  ros::Time::now(), distance_values);
                        }
                        break;
                        // 回冲状态发布  
                        case recharge:
                        {
                                int8_t recharge_status; 
                                if(!data_parse_ptr_->GetRechargeStatus(&recvData[0] + start_idx, recharge_status))
                                {
                                    break;
                                }
                                publishRechargeStatusTopic(recharge_pub_, recharge_status);  
                        }
                        break;
                        // 急停   
                        case stop:
                        {
                            int8_t stop_status; 
                            if(!data_parse_ptr_->GetStopStatus(&recvData[0] + start_idx, stop_status))
                            {
                                break;
                            }
                            publishStopStatusTopic(stop_pub_, stop_status);  
                        }
                        break;
                    }
                    start_idx += data_length;
                }
                delete [] recvData;    
                // 1us  延时  
                std::chrono::microseconds dura(1);
                std::this_thread::sleep_for(dura);
            }
        }
        
         //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         *  @brief 连接检查线程
         */
        void ConnectCheck()
        {
            while(1)
            {
                {
                    std::lock_guard<std::mutex> lock(connect_mutex_);
                    connect_count_++;
                    //std::cout<<"connect_count_: "<<connect_count_<<std::endl;
                    // 认为断开连接
                    if(connect_count_ >= 500)
                    {  
                        ROS_WARN("tcp connect OFF !!! ");
                        tcp_communicator_.SetConnectOff();  
                        ROS_INFO("try connect......." );
                        if(tcp_communicator_ .Connect(port_num_, net_addr_))
                        {
                            ROS_INFO("connect OK!!! " );
                        }
                        else
                        {
                            ROS_INFO("connect failure!!! " );
                        }
                        connect_count_ = 0;
                    }
                }
                // 1ms的延时  
                std::chrono::milliseconds dura(1);
                std::this_thread::sleep_for(dura);
            }
        }

    private:
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 运动控制话题回调处理函数
         */ 
        void motionHandler(const geometry_msgs::Twist &twist)
        {   
            // 读取线速度
            int16_t velocity = twist.linear.x * 1000;  // mm  
            // 读取角速度
            int16_t Angular = twist.angular.z * 1000;      // mrad 
            const uint8_t* speed_cmd_data = data_parse_ptr_->ConstructMotionCmd( velocity,  Angular);

            // 将速度命令通过网络发送出去 
            if(tcp_communicator_.GetConnectStatus())
            {
               tcp_communicator_.DataSend(speed_cmd_data, 9); 
               // std::cout<<"velocity: "<<velocity<<"  mm/s,  Angular: "<<Angular<<"  mrad/s"<<std::endl;
            }
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 回充处理函数
         */ 
        void rechargeHandler(const std_msgs::Int8 &recharge_cmd)
        {
            int8_t data = recharge_cmd.data;
            const uint8_t* recharge_cmd_data;  
            // 1 为充电任务下发
            if(data == 1)
            {
               recharge_cmd_data = data_parse_ptr_->ConstructRechargeCmd(1);  
            }
            else if(data == 0)  // 结束充电任务
            {
                recharge_cmd_data = data_parse_ptr_->ConstructRechargeCmd(0);  
            }

            // 将命令通过网络发送出去 
            if(tcp_communicator_.GetConnectStatus())
            {
               tcp_communicator_.DataSend(recharge_cmd_data, 4); 
               // std::cout<<"velocity: "<<velocity<<"  mm/s,  Angular: "<<Angular<<"  mrad/s"<<std::endl;
            }

            return;  
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        void publicImuTopic(const IMU &imu_data) const
        {
            sensor_msgs::Imu imu_msg;
            //欧拉角转四元数，其中欧拉角是rpy，对应旋转顺序为ZYX
            geometry_msgs::Quaternion quat( tf::createQuaternionMsgFromRollPitchYaw( imu_data.roll ,  imu_data.pitch,  imu_data.yaw ) ); 
            imu_msg.orientation = quat;
            imu_msg.linear_acceleration.x = imu_data.accel[0];
            imu_msg.linear_acceleration.y = imu_data.accel[1];
            imu_msg.linear_acceleration.z = imu_data.accel[2];
            imu_msg.angular_velocity.x = imu_data.gyro[0];
            imu_msg.angular_velocity.y = imu_data.gyro[1];
            imu_msg.angular_velocity.z = imu_data.gyro[2];

            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "imu_link";
            imu_pub_.publish(imu_msg);
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 发布超声波话题 
         */
        void publishUltrasonicTopic(ros::Publisher &ultrasonic_pub,  const ros::Time& stamp, std::vector<uint16_t> &distance_values)
        {
            cruiser_msgs::cruiserSensorAltrasonic altrasonic_msg;  

            if(distance_values.size() < 6)
            {
                for(uint8_t i = distance_values.size(); i < 6; i++)
                {
                    distance_values.push_back(0);  
                }
            }
                    
            altrasonic_msg.distance1 = distance_values[0];
            altrasonic_msg.distance2 = distance_values[1];
            altrasonic_msg.distance3 = distance_values[2];
            altrasonic_msg.distance4 = distance_values[3];
            altrasonic_msg.distance5 = distance_values[4];
            altrasonic_msg.distance6 = distance_values[5];

            ultrasonic_pub.publish(altrasonic_msg);
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 发布回充状态  
         */
        void publishRechargeStatusTopic(ros::Publisher &recharge_pub,  const int8_t &recharge_status)
        {
            std_msgs::Int8 recharge_status_msg;  

            if(90 == recharge_status)
            {
                recharge_status_ = RECHARGE_START_RUNNING;
            }
            else if (91 == recharge_status)
            {
                recharge_status_ = RECHARGE_FINISH_RUNNING;
            }
            else 
                recharge_status_ = RECHARGE_UNKNOWN;

            recharge_status_msg.data = recharge_status;  
            recharge_pub.publish(recharge_status_msg);
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 回充状态服务  
         */
        bool getRechargeStatusService(robot_drive::RechargeStatus::Request &req, robot_drive::RechargeStatus::Response &resp)
        {
            switch (recharge_status_) 
            {
                case RECHARGE_START_RUNNING:
                    resp.rechargeStatus.data = 1;   // start_running 正在上桩
                break;
                case RECHARGE_FINISH_RUNNING:
                    resp.rechargeStatus.data = 2;   // finish_running 正在下桩
                break;
                // case RECHARGE_FINISHED:
                // resp.rechargeStatus.data = 3;   // finished
                // break;
                // case RECHARGE_NO_PILE:
                // resp.rechargeStatus.data = 4;   // no pile
                // break;
                default:
                    resp.rechargeStatus.data = 0;   // unknown
                break;
            }
            return true;
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 发布急停状态  
         */
        void publishStopStatusTopic(ros::Publisher &stop_pub,  const int8_t &stop_status)
        {
            std_msgs::Int8 stop_status_msg;  

            if(stop_status != 0 && stop_status != 1 && stop_status != 2)
            {
                return;  
            }
            if(1 == stop_status)
            {
                stop_status_ = STOP_SOFT;
            }
            else if(2 == stop_status)
            {
                stop_status_ = STOP_HARD;
            }
            else
            {
                stop_status_ = STOP_IDLE;
            }
            
            stop_status_msg.data = stop_status;  
            stop_pub.publish(stop_status_msg);
        }

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * @brief 急停状态服务  
         */
        bool getStopStatusService(robot_drive::StopStatus::Request &req, robot_drive::StopStatus::Response &resp)
        {
            switch (stop_status_) 
            {
                case STOP_SOFT:
                    resp.stopStatus.data = 1;   // STOP_HARD 软急停
                break;
                case STOP_HARD:
                    resp.stopStatus.data = 2;   // STOP_SOFT 硬急停
                break;
                default:
                    resp.stopStatus.data = 0;   // STOP_IDLE 无急停
                break;
            }
            return true;
        }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    // glog 设置
    std::string ADDR = "/home/dt"; 
    //FLAGS_log_dir = ADDR + "/Log";
    //FLAGS_alsologtostderr = 1;     
    //google::InitGoogleLogging(argv[0]);
    //LOG(INFO)<<"Started RobotDrive_node";  
    //***** 初始化ROS *****/
    ros::init(argc, argv, "RobotDrive_node");
    std::string errmsg;
    bool ubt_key_check = false;// = check_key(errmsg);
    int ubt_key_check_cnt = 0;
    
    // while(!ubt_key_check)
    // {
    //      ubt_key_check = check_key(errmsg);
    //      ROS_WARN("RobotDrive_node UBT KEY Check Fail\n");
    //      ubt_key_check_cnt++;
    //      if (ubt_key_check_cnt > 4)
    //      {
    //          ROS_WARN("RobotDrive_node UBT KEY Check Fail, close node.\n");
    //          return -1 ;
    //      }
    // }

    //ROS_WARN("RobotDrive_node UBT KEY Check Success\n");
    ROS_WARN("Started RobotDrive_node");  
    RobotDriveNode robot_drive_node{5000, "10.7.6.51", 100 ,std::make_unique<DataParseChaoWei>()}; 
    // 数据接收处理线程
    std::thread data_parse{&RobotDriveNode::DataParse, &robot_drive_node};
    // 数据接收线程
    std::thread data_receive{&RobotDriveNode::DataRecived, &robot_drive_node};
    //  判断连接线程
    std::thread connect_check{&RobotDriveNode::ConnectCheck, &robot_drive_node};

    ros::spin();
    return 0;
}
