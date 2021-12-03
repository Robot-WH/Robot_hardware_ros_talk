#ifndef ROS_UTILS_HPP
#define ROS_UTILS_HPP


#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include "cruiserSensorAltrasonic.h"

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/TransformStamped.h>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 发布odom数据 
 * @details 应用于2D
 * @param odom_pub odom话题发布器
 * @param x, y  单位m
 * @param theta 单位弧度
 */

void publishOdomTopic(ros::Publisher &odom_pub, tf::TransformBroadcaster &odom_broadcaster, const float &x, const float &y, const float &theta, const float &velocity, const float &angular,
                                                    const ros::Time& stamp, const std::string &odom_frame_id,  const std::string &base_frame_id)
{
    // 由角度创建四元数    pitch = 0, roll = 0, yaw = theta;  
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    // publish the odom                发布当前帧里程计到/odom话题                 
    nav_msgs::Odometry odom;                            
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id;        // odom坐标   
    odom.child_frame_id = base_frame_id;        // 机体坐标
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = odom_quat;  
    // 速度
    odom.twist.twist.linear.x = velocity;
    odom.twist.twist.linear.y = 0; //velocity * angular / 45;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = angular;
    odom_pub.publish(odom);
    //std::cout << "odometry published" << std::endl;
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = odom_frame_id;  //odom信息所在的坐标系
	odom_trans.child_frame_id = base_frame_id;//"odom" ->  "base_footprint"
    odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;//odom位姿
    odom_broadcaster.sendTransform(odom_trans);
    //std::cout << "odometry tf published" << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T>
static double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T>
static void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T>
static void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename T>
static void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief convert Eigen::Matrix to geometry_msgs::TransformStamped  Eigen转换为tf msg
 * @param stamp            timestamp
 * @param pose             Eigen::Matrix to be converted
 * @param frame_id         tf frame_id
 * @param child_frame_id   tf child frame_id
 * @return converted TransformStamped
 */
static geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, 
                                                        const std::string& frame_id, const std::string& child_frame_id) 
{
  // 旋转矩阵 -> 四元数
  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  // 四元数单位化
  quat.normalize();
  // 构造四元数   ROS信息
  geometry_msgs::Quaternion odom_quat;
  odom_quat.w = quat.w();
  odom_quat.x = quat.x();
  odom_quat.y = quat.y();
  odom_quat.z = quat.z();
  
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = stamp;
  // 该tf关系表示 从 frame_id-> child_frame_id
  odom_trans.header.frame_id = frame_id;       
  odom_trans.child_frame_id = child_frame_id;

  odom_trans.transform.translation.x = pose(0, 3);
  odom_trans.transform.translation.y = pose(1, 3);
  odom_trans.transform.translation.z = pose(2, 3);
  odom_trans.transform.rotation = odom_quat;

  return odom_trans;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 输入: 位姿的ROS Msg
// 输出: Eigen变换矩阵
static Eigen::Isometry3d odom2isometry(const nav_msgs::OdometryConstPtr& odom_msg) 
{
  const auto& orientation = odom_msg->pose.pose.orientation;  
  const auto& position = odom_msg->pose.pose.position;
  // ROS   四元数转Eigen
  Eigen::Quaterniond quat;
  quat.w() = orientation.w;
  quat.x() = orientation.x;
  quat.y() = orientation.y;
  quat.z() = orientation.z;

  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  // linear是获取旋转矩阵
  isometry.linear() = quat.toRotationMatrix();
  // 赋值平移
  isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  return isometry;
}    
#endif // ROS_UTILS_HPP
