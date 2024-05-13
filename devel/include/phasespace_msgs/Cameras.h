// Generated by gencpp from file phasespace_msgs/Cameras.msg
// DO NOT EDIT!


#ifndef PHASESPACE_MSGS_MESSAGE_CAMERAS_H
#define PHASESPACE_MSGS_MESSAGE_CAMERAS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <phasespace_msgs/Camera.h>

namespace phasespace_msgs
{
template <class ContainerAllocator>
struct Cameras_
{
  typedef Cameras_<ContainerAllocator> Type;

  Cameras_()
    : header()
    , cameras()  {
    }
  Cameras_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , cameras(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::phasespace_msgs::Camera_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::phasespace_msgs::Camera_<ContainerAllocator> >> _cameras_type;
  _cameras_type cameras;





  typedef boost::shared_ptr< ::phasespace_msgs::Cameras_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::phasespace_msgs::Cameras_<ContainerAllocator> const> ConstPtr;

}; // struct Cameras_

typedef ::phasespace_msgs::Cameras_<std::allocator<void> > Cameras;

typedef boost::shared_ptr< ::phasespace_msgs::Cameras > CamerasPtr;
typedef boost::shared_ptr< ::phasespace_msgs::Cameras const> CamerasConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::phasespace_msgs::Cameras_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::phasespace_msgs::Cameras_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::phasespace_msgs::Cameras_<ContainerAllocator1> & lhs, const ::phasespace_msgs::Cameras_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.cameras == rhs.cameras;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::phasespace_msgs::Cameras_<ContainerAllocator1> & lhs, const ::phasespace_msgs::Cameras_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace phasespace_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::phasespace_msgs::Cameras_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::phasespace_msgs::Cameras_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::phasespace_msgs::Cameras_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::phasespace_msgs::Cameras_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::phasespace_msgs::Cameras_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::phasespace_msgs::Cameras_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::phasespace_msgs::Cameras_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5f4b3dddb1243eb3e913cbf3e1940fe8";
  }

  static const char* value(const ::phasespace_msgs::Cameras_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5f4b3dddb1243eb3ULL;
  static const uint64_t static_value2 = 0xe913cbf3e1940fe8ULL;
};

template<class ContainerAllocator>
struct DataType< ::phasespace_msgs::Cameras_<ContainerAllocator> >
{
  static const char* value()
  {
    return "phasespace_msgs/Cameras";
  }

  static const char* value(const ::phasespace_msgs::Cameras_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::phasespace_msgs::Cameras_<ContainerAllocator> >
{
  static const char* value()
  {
    return "########################################\n"
"# Messages\n"
"########################################\n"
"std_msgs/Header header\n"
"Camera[] cameras\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: phasespace_msgs/Camera\n"
"########################################\n"
"# Messages\n"
"########################################\n"
"uint32 id\n"
"uint32 flags\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
"float32 qw\n"
"float32 qx\n"
"float32 qy\n"
"float32 qz\n"
"float32 cond\n"
;
  }

  static const char* value(const ::phasespace_msgs::Cameras_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::phasespace_msgs::Cameras_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.cameras);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Cameras_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::phasespace_msgs::Cameras_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::phasespace_msgs::Cameras_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "cameras[]" << std::endl;
    for (size_t i = 0; i < v.cameras.size(); ++i)
    {
      s << indent << "  cameras[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::phasespace_msgs::Camera_<ContainerAllocator> >::stream(s, indent + "    ", v.cameras[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PHASESPACE_MSGS_MESSAGE_CAMERAS_H