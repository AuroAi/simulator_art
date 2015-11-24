// Generated by gencpp from file art_msgs/SteeringState.msg
// DO NOT EDIT!


#ifndef ART_MSGS_MESSAGE_STEERINGSTATE_H
#define ART_MSGS_MESSAGE_STEERINGSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <art_msgs/DriverState.h>

namespace art_msgs
{
template <class ContainerAllocator>
struct SteeringState_ {
  typedef SteeringState_<ContainerAllocator> Type;

  SteeringState_()
    : header()
  , driver()
  , angle(0.0)
  , sensor(0.0) {
  }
  SteeringState_(const ContainerAllocator& _alloc)
    : header(_alloc)
  , driver(_alloc)
  , angle(0.0)
  , sensor(0.0) {
  }



  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

  typedef  ::art_msgs::DriverState_<ContainerAllocator>  _driver_type;
  _driver_type driver;

  typedef float _angle_type;
  _angle_type angle;

  typedef float _sensor_type;
  _sensor_type sensor;




  typedef boost::shared_ptr< ::art_msgs::SteeringState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::art_msgs::SteeringState_<ContainerAllocator> const> ConstPtr;

}; // struct SteeringState_

typedef ::art_msgs::SteeringState_<std::allocator<void> > SteeringState;

typedef boost::shared_ptr< ::art_msgs::SteeringState > SteeringStatePtr;
typedef boost::shared_ptr< ::art_msgs::SteeringState const> SteeringStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::art_msgs::SteeringState_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::art_msgs::SteeringState_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::art_msgs::SteeringState_<ContainerAllocator> >
  : FalseType
{ };

template <class ContainerAllocator>
struct IsFixedSize< ::art_msgs::SteeringState_<ContainerAllocator> const>
  : FalseType
{ };

template <class ContainerAllocator>
struct IsMessage< ::art_msgs::SteeringState_<ContainerAllocator> >
  : TrueType
{ };

template <class ContainerAllocator>
struct IsMessage< ::art_msgs::SteeringState_<ContainerAllocator> const>
  : TrueType
{ };

template <class ContainerAllocator>
struct HasHeader< ::art_msgs::SteeringState_<ContainerAllocator> >
  : TrueType
{ };

template <class ContainerAllocator>
struct HasHeader< ::art_msgs::SteeringState_<ContainerAllocator> const>
  : TrueType
{ };


template<class ContainerAllocator>
struct MD5Sum< ::art_msgs::SteeringState_<ContainerAllocator> > {
  static const char* value() {
    return "7bf11da138f80579d285d99bea47f6d3";
  }

  static const char* value(const ::art_msgs::SteeringState_<ContainerAllocator>&) {
    return value();
  }
  static const uint64_t static_value1 = 0x7bf11da138f80579ULL;
  static const uint64_t static_value2 = 0xd285d99bea47f6d3ULL;
};

template<class ContainerAllocator>
struct DataType< ::art_msgs::SteeringState_<ContainerAllocator> > {
  static const char* value() {
    return "art_msgs/SteeringState";
  }

  static const char* value(const ::art_msgs::SteeringState_<ContainerAllocator>&) {
    return value();
  }
};

template<class ContainerAllocator>
struct Definition< ::art_msgs::SteeringState_<ContainerAllocator> > {
  static const char* value() {
    return "# ART steering controller state message\n\
\n\
# $Id$\n\
\n\
Header  header\n\
\n\
DriverState driver              # driver state\n\
float32 angle                   # steering angle in degrees\n\
float32 sensor                  # steering sensor voltage\n\
\n\
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
\n\
================================================================================\n\
MSG: art_msgs/DriverState\n\
# ART driver states -- similar to those in driver_base.\n\
\n\
# $Id$\n\
\n\
# constants\n\
uint32 CLOSED = 0       # Not connected to the hardware\n\
uint32 OPENED = 1       # Passively connected to the hardware\n\
uint32 RUNNING = 2      # Sending hardware commands\n\
\n\
uint32 state\n\
";
         }

           static const char* value(const ::art_msgs::SteeringState_<ContainerAllocator>&) {
           return value();
         }
         };

         } // namespace message_traits
         } // namespace ros

           namespace ros
           {
           namespace serialization
           {

           template<class ContainerAllocator> struct Serializer< ::art_msgs::SteeringState_<ContainerAllocator> > {
           template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m) {
           stream.next(m.header);
           stream.next(m.driver);
           stream.next(m.angle);
           stream.next(m.sensor);
         }

           ROS_DECLARE_ALLINONE_SERIALIZER;
         }; // struct SteeringState_

         } // namespace serialization
         } // namespace ros

           namespace ros
           {
           namespace message_operations
           {

           template<class ContainerAllocator>
           struct Printer< ::art_msgs::SteeringState_<ContainerAllocator> > {
           template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::art_msgs::SteeringState_<ContainerAllocator>& v) {
       s << indent << "header: ";
           s << std::endl;
           Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
       s << indent << "driver: ";
           s << std::endl;
           Printer< ::art_msgs::DriverState_<ContainerAllocator> >::stream(s, indent + "  ", v.driver);
       s << indent << "angle: ";
           Printer<float>::stream(s, indent + "  ", v.angle);
       s << indent << "sensor: ";
           Printer<float>::stream(s, indent + "  ", v.sensor);
         }
         };

         } // namespace message_operations
         } // namespace ros

#endif // ART_MSGS_MESSAGE_STEERINGSTATE_H
