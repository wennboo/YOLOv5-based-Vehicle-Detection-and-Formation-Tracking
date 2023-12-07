// Generated by gencpp from file robot_follow_yolo/anchor_box.msg
// DO NOT EDIT!


#ifndef ROBOT_FOLLOW_YOLO_MESSAGE_ANCHOR_BOX_H
#define ROBOT_FOLLOW_YOLO_MESSAGE_ANCHOR_BOX_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robot_follow_yolo
{
template <class ContainerAllocator>
struct anchor_box_
{
  typedef anchor_box_<ContainerAllocator> Type;

  anchor_box_()
    : x1(0.0)
    , y1(0.0)
    , x2(0.0)
    , y2(0.0)
    , proportion(0.0)
    , classes(0.0)  {
    }
  anchor_box_(const ContainerAllocator& _alloc)
    : x1(0.0)
    , y1(0.0)
    , x2(0.0)
    , y2(0.0)
    , proportion(0.0)
    , classes(0.0)  {
  (void)_alloc;
    }



   typedef float _x1_type;
  _x1_type x1;

   typedef float _y1_type;
  _y1_type y1;

   typedef float _x2_type;
  _x2_type x2;

   typedef float _y2_type;
  _y2_type y2;

   typedef float _proportion_type;
  _proportion_type proportion;

   typedef float _classes_type;
  _classes_type classes;





  typedef boost::shared_ptr< ::robot_follow_yolo::anchor_box_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_follow_yolo::anchor_box_<ContainerAllocator> const> ConstPtr;

}; // struct anchor_box_

typedef ::robot_follow_yolo::anchor_box_<std::allocator<void> > anchor_box;

typedef boost::shared_ptr< ::robot_follow_yolo::anchor_box > anchor_boxPtr;
typedef boost::shared_ptr< ::robot_follow_yolo::anchor_box const> anchor_boxConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_follow_yolo::anchor_box_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_follow_yolo::anchor_box_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robot_follow_yolo::anchor_box_<ContainerAllocator1> & lhs, const ::robot_follow_yolo::anchor_box_<ContainerAllocator2> & rhs)
{
  return lhs.x1 == rhs.x1 &&
    lhs.y1 == rhs.y1 &&
    lhs.x2 == rhs.x2 &&
    lhs.y2 == rhs.y2 &&
    lhs.proportion == rhs.proportion &&
    lhs.classes == rhs.classes;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robot_follow_yolo::anchor_box_<ContainerAllocator1> & lhs, const ::robot_follow_yolo::anchor_box_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robot_follow_yolo

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::robot_follow_yolo::anchor_box_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_follow_yolo::anchor_box_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_follow_yolo::anchor_box_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_follow_yolo::anchor_box_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_follow_yolo::anchor_box_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_follow_yolo::anchor_box_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_follow_yolo::anchor_box_<ContainerAllocator> >
{
  static const char* value()
  {
    return "961bea4c7f3887f07068935c6ef2d06e";
  }

  static const char* value(const ::robot_follow_yolo::anchor_box_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x961bea4c7f3887f0ULL;
  static const uint64_t static_value2 = 0x7068935c6ef2d06eULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_follow_yolo::anchor_box_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_follow_yolo/anchor_box";
  }

  static const char* value(const ::robot_follow_yolo::anchor_box_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_follow_yolo::anchor_box_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 x1\n"
"float32 y1\n"
"float32 x2\n"
"float32 y2\n"
"float32 proportion\n"
"float32 classes\n"
"\n"
"\n"
;
  }

  static const char* value(const ::robot_follow_yolo::anchor_box_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_follow_yolo::anchor_box_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x1);
      stream.next(m.y1);
      stream.next(m.x2);
      stream.next(m.y2);
      stream.next(m.proportion);
      stream.next(m.classes);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct anchor_box_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_follow_yolo::anchor_box_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robot_follow_yolo::anchor_box_<ContainerAllocator>& v)
  {
    s << indent << "x1: ";
    Printer<float>::stream(s, indent + "  ", v.x1);
    s << indent << "y1: ";
    Printer<float>::stream(s, indent + "  ", v.y1);
    s << indent << "x2: ";
    Printer<float>::stream(s, indent + "  ", v.x2);
    s << indent << "y2: ";
    Printer<float>::stream(s, indent + "  ", v.y2);
    s << indent << "proportion: ";
    Printer<float>::stream(s, indent + "  ", v.proportion);
    s << indent << "classes: ";
    Printer<float>::stream(s, indent + "  ", v.classes);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_FOLLOW_YOLO_MESSAGE_ANCHOR_BOX_H