// Generated by gencpp from file art_msgs/GpsInfo.msg
// DO NOT EDIT!


#ifndef ART_MSGS_MESSAGE_GPSINFO_H
#define ART_MSGS_MESSAGE_GPSINFO_H


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
struct GpsInfo_
{
  typedef GpsInfo_<ContainerAllocator> Type;

  GpsInfo_()
    : header()
    , latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , utm_e(0.0)
    , utm_n(0.0)
    , zone()
    , hdop(0.0)
    , vdop(0.0)
    , err_horz(0.0)
    , err_vert(0.0)
    , quality(0)
    , num_sats(0)
  {
  }
  GpsInfo_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , utm_e(0.0)
    , utm_n(0.0)
    , zone(_alloc)
    , hdop(0.0)
    , vdop(0.0)
    , err_horz(0.0)
    , err_vert(0.0)
    , quality(0)
    , num_sats(0)
  {
  }



  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

  typedef double _latitude_type;
  _latitude_type latitude;

  typedef double _longitude_type;
  _longitude_type longitude;

  typedef double _altitude_type;
  _altitude_type altitude;

  typedef double _utm_e_type;
  _utm_e_type utm_e;

  typedef double _utm_n_type;
  _utm_n_type utm_n;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _zone_type;
  _zone_type zone;

  typedef double _hdop_type;
  _hdop_type hdop;

  typedef double _vdop_type;
  _vdop_type vdop;

  typedef double _err_horz_type;
  _err_horz_type err_horz;

  typedef double _err_vert_type;
  _err_vert_type err_vert;

  typedef uint16_t _quality_type;
  _quality_type quality;

  typedef uint16_t _num_sats_type;
  _num_sats_type num_sats;


  enum { INVALID_FIX = 0u };
  enum { GPS_FIX = 1u };
  enum { DGPS_FIX = 2u };


  typedef boost::shared_ptr< ::art_msgs::GpsInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::art_msgs::GpsInfo_<ContainerAllocator> const> ConstPtr;

}; // struct GpsInfo_

typedef ::art_msgs::GpsInfo_<std::allocator<void> > GpsInfo;

typedef boost::shared_ptr< ::art_msgs::GpsInfo > GpsInfoPtr;
typedef boost::shared_ptr< ::art_msgs::GpsInfo const> GpsInfoConstPtr;

// constants requiring out of line definition









template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::art_msgs::GpsInfo_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::art_msgs::GpsInfo_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::art_msgs::GpsInfo_<ContainerAllocator> >
    : FalseType
{ };

template <class ContainerAllocator>
struct IsFixedSize< ::art_msgs::GpsInfo_<ContainerAllocator> const>
    : FalseType
{ };

template <class ContainerAllocator>
struct IsMessage< ::art_msgs::GpsInfo_<ContainerAllocator> >
    : TrueType
{ };

template <class ContainerAllocator>
struct IsMessage< ::art_msgs::GpsInfo_<ContainerAllocator> const>
    : TrueType
{ };

template <class ContainerAllocator>
struct HasHeader< ::art_msgs::GpsInfo_<ContainerAllocator> >
    : TrueType
{ };

template <class ContainerAllocator>
struct HasHeader< ::art_msgs::GpsInfo_<ContainerAllocator> const>
    : TrueType
{ };


template<class ContainerAllocator>
struct MD5Sum< ::art_msgs::GpsInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4f5e197f8744c1a11f1c94dc6e9a77a6";
  }

  static const char* value(const ::art_msgs::GpsInfo_<ContainerAllocator>&)
  {
    return value();
  }
  static const uint64_t static_value1 = 0x4f5e197f8744c1a1ULL;
  static const uint64_t static_value2 = 0x1f1c94dc6e9a77a6ULL;
};

template<class ContainerAllocator>
struct DataType< ::art_msgs::GpsInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "art_msgs/GpsInfo";
  }

  static const char* value(const ::art_msgs::GpsInfo_<ContainerAllocator>&)
  {
    return value();
  }
};

template<class ContainerAllocator>
struct Definition< ::art_msgs::GpsInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# GPS position message\n\
#\n\
# Probably to be replaced by a standard ROS message for Diamondback.\n\
\n\
# $Id$\n\
\n\
# standard ROS header, includes time stamp\n\
Header header\n\
\n\
# Latitude in degrees.  Positive is north of equator, negative is\n\
# south of equator.\n\
float64 latitude\n\
\n\
# Longitude in degrees.  Positive is east of prime meridian, negative\n\
# is west of prime meridian.\n\
float64 longitude\n\
\n\
# Altitude, in meters.  Positive is above reference (e.g., sea-level),\n\
# and negative is below.\n\
float64 altitude\n\
\n\
# UTM WGS84 coordinates, easting [m]\n\
float64 utm_e\n\
\n\
# UTM WGS84 coordinates, northing [m]\n\
float64 utm_n\n\
\n\
# UTM zone\n\
string zone\n\
\n\
# Horizontal dilution of position (HDOP)\n\
float64 hdop\n\
\n\
# Vertical dilution of position (VDOP)\n\
float64 vdop\n\
\n\
# Horizonal error [m]\n\
float64 err_horz\n\
\n\
# Vertical error [m]\n\
float64 err_vert\n\
\n\
# Quality of fix 0 = invalid, 1 = GPS fix, 2 = Differential GPS fix\n\
uint16 INVALID_FIX = 0\n\
uint16 GPS_FIX = 1\n\
uint16 DGPS_FIX = 2\n\
uint16 quality\n\
\n\
# Number of satellites in view.\n\
uint16 num_sats\n\
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

  static const char* value(const ::art_msgs::GpsInfo_<ContainerAllocator>&)
  {
    return value();
  }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::art_msgs::GpsInfo_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.latitude);
    stream.next(m.longitude);
    stream.next(m.altitude);
    stream.next(m.utm_e);
    stream.next(m.utm_n);
    stream.next(m.zone);
    stream.next(m.hdop);
    stream.next(m.vdop);
    stream.next(m.err_horz);
    stream.next(m.err_vert);
    stream.next(m.quality);
    stream.next(m.num_sats);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct GpsInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::art_msgs::GpsInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::art_msgs::GpsInfo_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "latitude: ";
    Printer<double>::stream(s, indent + "  ", v.latitude);
    s << indent << "longitude: ";
    Printer<double>::stream(s, indent + "  ", v.longitude);
    s << indent << "altitude: ";
    Printer<double>::stream(s, indent + "  ", v.altitude);
    s << indent << "utm_e: ";
    Printer<double>::stream(s, indent + "  ", v.utm_e);
    s << indent << "utm_n: ";
    Printer<double>::stream(s, indent + "  ", v.utm_n);
    s << indent << "zone: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.zone);
    s << indent << "hdop: ";
    Printer<double>::stream(s, indent + "  ", v.hdop);
    s << indent << "vdop: ";
    Printer<double>::stream(s, indent + "  ", v.vdop);
    s << indent << "err_horz: ";
    Printer<double>::stream(s, indent + "  ", v.err_horz);
    s << indent << "err_vert: ";
    Printer<double>::stream(s, indent + "  ", v.err_vert);
    s << indent << "quality: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.quality);
    s << indent << "num_sats: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.num_sats);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ART_MSGS_MESSAGE_GPSINFO_H
