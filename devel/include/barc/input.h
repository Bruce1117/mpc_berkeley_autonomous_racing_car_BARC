// Generated by gencpp from file barc/input.msg
// DO NOT EDIT!


#ifndef BARC_MESSAGE_INPUT_H
#define BARC_MESSAGE_INPUT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace barc
{
template <class ContainerAllocator>
struct input_
{
  typedef input_<ContainerAllocator> Type;

  input_()
    : accel(0.0)
    , steer(0.0)  {
    }
  input_(const ContainerAllocator& _alloc)
    : accel(0.0)
    , steer(0.0)  {
  (void)_alloc;
    }



   typedef double _accel_type;
  _accel_type accel;

   typedef double _steer_type;
  _steer_type steer;





  typedef boost::shared_ptr< ::barc::input_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::barc::input_<ContainerAllocator> const> ConstPtr;

}; // struct input_

typedef ::barc::input_<std::allocator<void> > input;

typedef boost::shared_ptr< ::barc::input > inputPtr;
typedef boost::shared_ptr< ::barc::input const> inputConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::barc::input_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::barc::input_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::barc::input_<ContainerAllocator1> & lhs, const ::barc::input_<ContainerAllocator2> & rhs)
{
  return lhs.accel == rhs.accel &&
    lhs.steer == rhs.steer;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::barc::input_<ContainerAllocator1> & lhs, const ::barc::input_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace barc

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::barc::input_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::barc::input_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::barc::input_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::barc::input_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::barc::input_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::barc::input_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::barc::input_<ContainerAllocator> >
{
  static const char* value()
  {
    return "be360b2b21a0fff5bfe4192d75a591ef";
  }

  static const char* value(const ::barc::input_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbe360b2b21a0fff5ULL;
  static const uint64_t static_value2 = 0xbfe4192d75a591efULL;
};

template<class ContainerAllocator>
struct DataType< ::barc::input_<ContainerAllocator> >
{
  static const char* value()
  {
    return "barc/input";
  }

  static const char* value(const ::barc::input_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::barc::input_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 accel\n"
"float64 steer\n"
;
  }

  static const char* value(const ::barc::input_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::barc::input_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.accel);
      stream.next(m.steer);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct input_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::barc::input_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::barc::input_<ContainerAllocator>& v)
  {
    s << indent << "accel: ";
    Printer<double>::stream(s, indent + "  ", v.accel);
    s << indent << "steer: ";
    Printer<double>::stream(s, indent + "  ", v.steer);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BARC_MESSAGE_INPUT_H