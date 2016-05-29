// Generated by gencpp from file art_msgs/Shifter.msg
// DO NOT EDIT!

#ifndef ART_MSGS_MESSAGE_SHIFTER_H
#define ART_MSGS_MESSAGE_SHIFTER_H

#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace art_msgs
{
template <class ContainerAllocator>
struct Shifter_
{
  typedef Shifter_<ContainerAllocator> Type;

  Shifter_() : header(), gear(0), relays(0)
  {
  }
  Shifter_(const ContainerAllocator &_alloc) : header(_alloc), gear(0), relays(0)
  {
  }

  typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
  _header_type header;

  typedef uint8_t _gear_type;
  _gear_type gear;

  typedef uint8_t _relays_type;
  _relays_type relays;

  enum
  {
    Reset = 0u
  };
  enum
  {
    Park = 1u
  };
  enum
  {
    Reverse = 2u
  };
  enum
  {
    Neutral = 3u
  };
  enum
  {
    Drive = 4u
  };

  typedef boost::shared_ptr<::art_msgs::Shifter_<ContainerAllocator>> Ptr;
  typedef boost::shared_ptr<::art_msgs::Shifter_<ContainerAllocator> const> ConstPtr;

};  // struct Shifter_

typedef ::art_msgs::Shifter_<std::allocator<void>> Shifter;

typedef boost::shared_ptr<::art_msgs::Shifter> ShifterPtr;
typedef boost::shared_ptr<::art_msgs::Shifter const> ShifterConstPtr;

// constants requiring out of line definition

template <typename ContainerAllocator>
std::ostream &operator<<(std::ostream &s, const ::art_msgs::Shifter_<ContainerAllocator> &v)
{
  ros::message_operations::Printer<::art_msgs::Shifter_<ContainerAllocator>>::stream(s, "", v);
  return s;
}

}  // namespace art_msgs

namespace ros
{
namespace message_traits
{
// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'nav_msgs': ['/opt/ros/indigo/share/nav_msgs/cmake/../msg'], 'sensor_msgs':
// ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'actionlib_msgs':
// ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'std_msgs':
// ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'art_msgs':
// ['/fast_data/ros/auro_repo/sandbox/src/map/art_msgs/msg'], 'geometry_msgs':
// ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__',
// '__format__', '__getattribute__', '__hash__', '__init__', '__module__',
// '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__',
// '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__',
// '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header',
// 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text',
// 'types']

template <class ContainerAllocator>
struct IsFixedSize<::art_msgs::Shifter_<ContainerAllocator>> : FalseType
{
};

template <class ContainerAllocator>
struct IsFixedSize<::art_msgs::Shifter_<ContainerAllocator> const> : FalseType
{
};

template <class ContainerAllocator>
struct IsMessage<::art_msgs::Shifter_<ContainerAllocator>> : TrueType
{
};

template <class ContainerAllocator>
struct IsMessage<::art_msgs::Shifter_<ContainerAllocator> const> : TrueType
{
};

template <class ContainerAllocator>
struct HasHeader<::art_msgs::Shifter_<ContainerAllocator>> : TrueType
{
};

template <class ContainerAllocator>
struct HasHeader<::art_msgs::Shifter_<ContainerAllocator> const> : TrueType
{
};

template <class ContainerAllocator>
struct MD5Sum<::art_msgs::Shifter_<ContainerAllocator>>
{
  static const char *value()
  {
    return "dddb61d8575e01ea7857ef1b3a7b941c";
  }

  static const char *value(const ::art_msgs::Shifter_<ContainerAllocator> &)
  {
    return value();
  }
  static const uint64_t static_value1 = 0xdddb61d8575e01eaULL;
  static const uint64_t static_value2 = 0x7857ef1b3a7b941cULL;
};

template <class ContainerAllocator>
struct DataType<::art_msgs::Shifter_<ContainerAllocator>>
{
  static const char *value()
  {
    return "art_msgs/Shifter";
  }

  static const char *value(const ::art_msgs::Shifter_<ContainerAllocator> &)
  {
    return value();
  }
};

template <class ContainerAllocator>
struct Definition<::art_msgs::Shifter_<ContainerAllocator>>
{
  static const char *value()
  {
    return "# ART shifter message\n\
#\n\
# Used to both request and report gear shifts.\n\
\n\
# $Id$\n\
\n\
# Our Arens Controls hardware mechanism requires holding the shift\n\
# relay on for one second before resetting it.  A command node must\n\
# request the desired gear, then wait a second before sending the\n\
# Reset command.  To be safe, it should check that the shift actually\n\
# occurred before continuing.\n\
\n\
Header  header          # standard ROS message header\n\
\n\
# gear numbers\n\
uint8 Reset = 0         # request reset of shifter relays\n\
uint8 Park = 1\n\
uint8 Reverse = 2\n\
uint8 Neutral = 3\n\
uint8 Drive = 4\n\
\n\
uint8 gear              # requested or reported gear number\n\
uint8 relays            # current relay values\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char *value(const ::art_msgs::Shifter_<ContainerAllocator> &)
  {
    return value();
  }
};

}  // namespace message_traits
}  // namespace ros

namespace ros
{
namespace serialization
{
template <class ContainerAllocator>
struct Serializer<::art_msgs::Shifter_<ContainerAllocator>>
{
  template <typename Stream, typename T>
  inline static void allInOne(Stream &stream, T m)
  {
    stream.next(m.header);
    stream.next(m.gear);
    stream.next(m.relays);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
};  // struct Shifter_

}  // namespace serialization
}  // namespace ros

namespace ros
{
namespace message_operations
{
template <class ContainerAllocator>
struct Printer<::art_msgs::Shifter_<ContainerAllocator>>
{
  template <typename Stream>
  static void stream(Stream &s, const std::string &indent, const ::art_msgs::Shifter_<ContainerAllocator> &v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer<::std_msgs::Header_<ContainerAllocator>>::stream(s, indent + "  ", v.header);
    s << indent << "gear: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gear);
    s << indent << "relays: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.relays);
  }
};

}  // namespace message_operations
}  // namespace ros

#endif  // ART_MSGS_MESSAGE_SHIFTER_H
