// Generated by gencpp from file robot_drive/StopStatusRequest.msg
// DO NOT EDIT!


#ifndef ROBOT_DRIVE_MESSAGE_STOPSTATUSREQUEST_H
#define ROBOT_DRIVE_MESSAGE_STOPSTATUSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robot_drive
{
template <class ContainerAllocator>
struct StopStatusRequest_
{
  typedef StopStatusRequest_<ContainerAllocator> Type;

  StopStatusRequest_()
    {
    }
  StopStatusRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::robot_drive::StopStatusRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_drive::StopStatusRequest_<ContainerAllocator> const> ConstPtr;

}; // struct StopStatusRequest_

typedef ::robot_drive::StopStatusRequest_<std::allocator<void> > StopStatusRequest;

typedef boost::shared_ptr< ::robot_drive::StopStatusRequest > StopStatusRequestPtr;
typedef boost::shared_ptr< ::robot_drive::StopStatusRequest const> StopStatusRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_drive::StopStatusRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_drive::StopStatusRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace robot_drive

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::robot_drive::StopStatusRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_drive::StopStatusRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_drive::StopStatusRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_drive::StopStatusRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_drive::StopStatusRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_drive::StopStatusRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_drive::StopStatusRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::robot_drive::StopStatusRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_drive::StopStatusRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_drive/StopStatusRequest";
  }

  static const char* value(const ::robot_drive::StopStatusRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_drive::StopStatusRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::robot_drive::StopStatusRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_drive::StopStatusRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StopStatusRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_drive::StopStatusRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::robot_drive::StopStatusRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_DRIVE_MESSAGE_STOPSTATUSREQUEST_H
