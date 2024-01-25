// Generated by gencpp from file barc/State.msg
// DO NOT EDIT!


#ifndef BARC_MESSAGE_STATE_H
#define BARC_MESSAGE_STATE_H


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
struct State_
{
  typedef State_<ContainerAllocator> Type;

  State_()
    : x(0.0)
    , y(0.0)
    , v(0.0)
    , psi(0.0)  {
    }
  State_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , v(0.0)
    , psi(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _v_type;
  _v_type v;

   typedef double _psi_type;
  _psi_type psi;





  typedef boost::shared_ptr< ::barc::State_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::barc::State_<ContainerAllocator> const> ConstPtr;

}; // struct State_

typedef ::barc::State_<std::allocator<void> > State;

typedef boost::shared_ptr< ::barc::State > StatePtr;
typedef boost::shared_ptr< ::barc::State const> StateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::barc::State_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::barc::State_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::barc::State_<ContainerAllocator1> & lhs, const ::barc::State_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.v == rhs.v &&
    lhs.psi == rhs.psi;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::barc::State_<ContainerAllocator1> & lhs, const ::barc::State_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace barc

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::barc::State_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::barc::State_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::barc::State_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::barc::State_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::barc::State_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::barc::State_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::barc::State_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fe3ac2b76adaa1950be1785133b8a2c8";
  }

  static const char* value(const ::barc::State_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfe3ac2b76adaa195ULL;
  static const uint64_t static_value2 = 0x0be1785133b8a2c8ULL;
};

template<class ContainerAllocator>
struct DataType< ::barc::State_<ContainerAllocator> >
{
  static const char* value()
  {
    return "barc/State";
  }

  static const char* value(const ::barc::State_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::barc::State_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x\n"
"float64 y\n"
"float64 v\n"
"float64 psi\n"
;
  }

  static const char* value(const ::barc::State_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::barc::State_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.v);
      stream.next(m.psi);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct State_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::barc::State_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::barc::State_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "v: ";
    Printer<double>::stream(s, indent + "  ", v.v);
    s << indent << "psi: ";
    Printer<double>::stream(s, indent + "  ", v.psi);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BARC_MESSAGE_STATE_H
