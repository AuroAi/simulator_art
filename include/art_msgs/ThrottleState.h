// Generated by gencpp from file art_msgs/ThrottleState.msg
// DO NOT EDIT!


#ifndef ART_MSGS_MESSAGE_THROTTLESTATE_H
#define ART_MSGS_MESSAGE_THROTTLESTATE_H


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
struct ThrottleState_ {
  typedef ThrottleState_<ContainerAllocator> Type;

  ThrottleState_()
    : header()
  , position(0.0)
  , rpms(0.0)
  , estop(0)
  , pwm(0.0)
  , dstate(0.0)
  , istate(0.0) {
  }
  ThrottleState_(const ContainerAllocator& _alloc)
    : header(_alloc)
  , position(0.0)
  , rpms(0.0)
  , estop(0)
  , pwm(0.0)
  , dstate(0.0)
  , istate(0.0) {
  }



  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

  typedef float _position_type;
  _position_type position;

  typedef float _rpms_type;
  _rpms_type rpms;

  typedef uint8_t _estop_type;
  _estop_type estop;

  typedef float _pwm_type;
  _pwm_type pwm;

  typedef float _dstate_type;
  _dstate_type dstate;

  typedef float _istate_type;
  _istate_type istate;




  typedef boost::shared_ptr< ::art_msgs::ThrottleState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::art_msgs::ThrottleState_<ContainerAllocator> const> ConstPtr;

}; // struct ThrottleState_

typedef ::art_msgs::ThrottleState_<std::allocator<void> > ThrottleState;

typedef boost::shared_ptr< ::art_msgs::ThrottleState > ThrottleStatePtr;
typedef boost::shared_ptr< ::art_msgs::ThrottleState const> ThrottleStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::art_msgs::ThrottleState_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::art_msgs::ThrottleState_<ContainerAllocator> >::stream(s, "", v);
  return s;
}

} // namespace art_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'nav_msgs': ['/opt/ros/indigo/share/nav_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'art_msgs': ['/fast_data/ros/auro_repo/sandbox/src/map/art_msgs/msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::art_msgs::ThrottleState_<ContainerAllocator> >
  : FalseType
{ };

template <class ContainerAllocator>
struct IsFixedSize< ::art_msgs::ThrottleState_<ContainerAllocator> const>
  : FalseType
{ };

template <class ContainerAllocator>
struct IsMessage< ::art_msgs::ThrottleState_<ContainerAllocator> >
  : TrueType
{ };

template <class ContainerAllocator>
struct IsMessage< ::art_msgs::ThrottleState_<ContainerAllocator> const>
  : TrueType
{ };

template <class ContainerAllocator>
struct HasHeader< ::art_msgs::ThrottleState_<ContainerAllocator> >
  : TrueType
{ };

template <class ContainerAllocator>
struct HasHeader< ::art_msgs::ThrottleState_<ContainerAllocator> const>
  : TrueType
{ };


template<class ContainerAllocator>
struct MD5Sum< ::art_msgs::ThrottleState_<ContainerAllocator> > {
  static const char* value() {
    return "5b1c64434e6ebe49625631607b188cd5";
  }

  static const char* value(const ::art_msgs::ThrottleState_<ContainerAllocator>&) {
    return value();
  }
  static const uint64_t static_value1 = 0x5b1c64434e6ebe49ULL;
  static const uint64_t static_value2 = 0x625631607b188cd5ULL;
};

template<class ContainerAllocator>
struct DataType< ::art_msgs::ThrottleState_<ContainerAllocator> > {
  static const char* value() {
    return "art_msgs/ThrottleState";
  }

  static const char* value(const ::art_msgs::ThrottleState_<ContainerAllocator>&) {
    return value();
  }
};

template<class ContainerAllocator>
struct Definition< ::art_msgs::ThrottleState_<ContainerAllocator> > {
  static const char* value() {
    return "# ART throttle controller state message\n\
\n\
# $Id$\n\
\n\
Header  header\n\
\n\
float32 position                # fractional position [0, 1]\n\
float32 rpms                    # engine speed (rev/min)\n\
uint8   estop                   # emergency stop indicator\n\
\n\
# optional extra diagnostic information:\n\
float32 pwm                     # Pulse Width Modulation value\n\
float32 dstate                  # PID derivative state\n\
float32 istate                  # PID integral state\n\
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

           static const char* value(const ::art_msgs::ThrottleState_<ContainerAllocator>&) {
           return value();
         }
         };

         } // namespace message_traits
         } // namespace ros

           namespace ros
           {
           namespace serialization
           {

           template<class ContainerAllocator> struct Serializer< ::art_msgs::ThrottleState_<ContainerAllocator> > {
           template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m) {
           stream.next(m.header);
           stream.next(m.position);
           stream.next(m.rpms);
           stream.next(m.estop);
           stream.next(m.pwm);
           stream.next(m.dstate);
           stream.next(m.istate);
         }

           ROS_DECLARE_ALLINONE_SERIALIZER;
         }; // struct ThrottleState_

         } // namespace serialization
         } // namespace ros

           namespace ros
           {
           namespace message_operations
           {

           template<class ContainerAllocator>
           struct Printer< ::art_msgs::ThrottleState_<ContainerAllocator> > {
           template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::art_msgs::ThrottleState_<ContainerAllocator>& v) {
       s << indent << "header: ";
           s << std::endl;
           Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
       s << indent << "position: ";
           Printer<float>::stream(s, indent + "  ", v.position);
       s << indent << "rpms: ";
           Printer<float>::stream(s, indent + "  ", v.rpms);
       s << indent << "estop: ";
           Printer<uint8_t>::stream(s, indent + "  ", v.estop);
       s << indent << "pwm: ";
           Printer<float>::stream(s, indent + "  ", v.pwm);
       s << indent << "dstate: ";
           Printer<float>::stream(s, indent + "  ", v.dstate);
       s << indent << "istate: ";
           Printer<float>::stream(s, indent + "  ", v.istate);
         }
         };

         } // namespace message_operations
         } // namespace ros

#endif // ART_MSGS_MESSAGE_THROTTLESTATE_H
