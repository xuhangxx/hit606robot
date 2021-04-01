// Generated by gencpp from file velodyne_datatest/velodyne_obstacle.msg
// DO NOT EDIT!


#ifndef VELODYNE_DATATEST_MESSAGE_VELODYNE_OBSTACLE_H
#define VELODYNE_DATATEST_MESSAGE_VELODYNE_OBSTACLE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace velodyne_datatest
{
template <class ContainerAllocator>
struct velodyne_obstacle_
{
  typedef velodyne_obstacle_<ContainerAllocator> Type;

  velodyne_obstacle_()
    : distance(0.0)
    , yaw_angle(0.0)  {
    }
  velodyne_obstacle_(const ContainerAllocator& _alloc)
    : distance(0.0)
    , yaw_angle(0.0)  {
  (void)_alloc;
    }



   typedef float _distance_type;
  _distance_type distance;

   typedef float _yaw_angle_type;
  _yaw_angle_type yaw_angle;





  typedef boost::shared_ptr< ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator> const> ConstPtr;

}; // struct velodyne_obstacle_

typedef ::velodyne_datatest::velodyne_obstacle_<std::allocator<void> > velodyne_obstacle;

typedef boost::shared_ptr< ::velodyne_datatest::velodyne_obstacle > velodyne_obstaclePtr;
typedef boost::shared_ptr< ::velodyne_datatest::velodyne_obstacle const> velodyne_obstacleConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace velodyne_datatest

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'velodyne_datatest': ['/home/robot/catkin_805/src/velodyne_datatest/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e3c352c49c90f146e3491d4a1f91f87d";
  }

  static const char* value(const ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe3c352c49c90f146ULL;
  static const uint64_t static_value2 = 0xe3491d4a1f91f87dULL;
};

template<class ContainerAllocator>
struct DataType< ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "velodyne_datatest/velodyne_obstacle";
  }

  static const char* value(const ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 distance\n\
float32 yaw_angle\n\
";
  }

  static const char* value(const ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.distance);
      stream.next(m.yaw_angle);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct velodyne_obstacle_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::velodyne_datatest::velodyne_obstacle_<ContainerAllocator>& v)
  {
    s << indent << "distance: ";
    Printer<float>::stream(s, indent + "  ", v.distance);
    s << indent << "yaw_angle: ";
    Printer<float>::stream(s, indent + "  ", v.yaw_angle);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VELODYNE_DATATEST_MESSAGE_VELODYNE_OBSTACLE_H
