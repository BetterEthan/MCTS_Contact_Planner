// Generated by gencpp from file hit_spider/hexapod_State.msg
// DO NOT EDIT!


#ifndef HIT_SPIDER_MESSAGE_HEXAPOD_STATE_H
#define HIT_SPIDER_MESSAGE_HEXAPOD_STATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <hit_spider/hexapod_Base_Pose.h>
#include <hit_spider/FeetPosition.h>
#include <hit_spider/hexapod_Base_Pose.h>
#include <hit_spider/FeetPosition.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <hit_spider/FeetNormalVector.h>

namespace hit_spider
{
template <class ContainerAllocator>
struct hexapod_State_
{
  typedef hexapod_State_<ContainerAllocator> Type;

  hexapod_State_()
    : base_Pose_Now()
    , support_State_Now()
    , faultLeg_State_Now()
    , feetPositionNow()
    , base_Pose_Next()
    , support_State_Next()
    , faultLeg_State_Next()
    , feetPositionNext()
    , move_Direction()
    , remarks()
    , feetNormalVector()
    , mu()
    , maxNormalF()  {
      support_State_Now.assign(0);

      faultLeg_State_Now.assign(0);

      support_State_Next.assign(0);

      faultLeg_State_Next.assign(0);

      mu.assign(0.0);

      maxNormalF.assign(0.0);
  }
  hexapod_State_(const ContainerAllocator& _alloc)
    : base_Pose_Now(_alloc)
    , support_State_Now()
    , faultLeg_State_Now()
    , feetPositionNow(_alloc)
    , base_Pose_Next(_alloc)
    , support_State_Next()
    , faultLeg_State_Next()
    , feetPositionNext(_alloc)
    , move_Direction(_alloc)
    , remarks(_alloc)
    , feetNormalVector(_alloc)
    , mu()
    , maxNormalF()  {
  (void)_alloc;
      support_State_Now.assign(0);

      faultLeg_State_Now.assign(0);

      support_State_Next.assign(0);

      faultLeg_State_Next.assign(0);

      mu.assign(0.0);

      maxNormalF.assign(0.0);
  }



   typedef  ::hit_spider::hexapod_Base_Pose_<ContainerAllocator>  _base_Pose_Now_type;
  _base_Pose_Now_type base_Pose_Now;

   typedef boost::array<int8_t, 6>  _support_State_Now_type;
  _support_State_Now_type support_State_Now;

   typedef boost::array<int8_t, 6>  _faultLeg_State_Now_type;
  _faultLeg_State_Now_type faultLeg_State_Now;

   typedef  ::hit_spider::FeetPosition_<ContainerAllocator>  _feetPositionNow_type;
  _feetPositionNow_type feetPositionNow;

   typedef  ::hit_spider::hexapod_Base_Pose_<ContainerAllocator>  _base_Pose_Next_type;
  _base_Pose_Next_type base_Pose_Next;

   typedef boost::array<int8_t, 6>  _support_State_Next_type;
  _support_State_Next_type support_State_Next;

   typedef boost::array<int8_t, 6>  _faultLeg_State_Next_type;
  _faultLeg_State_Next_type faultLeg_State_Next;

   typedef  ::hit_spider::FeetPosition_<ContainerAllocator>  _feetPositionNext_type;
  _feetPositionNext_type feetPositionNext;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _move_Direction_type;
  _move_Direction_type move_Direction;

   typedef  ::std_msgs::String_<ContainerAllocator>  _remarks_type;
  _remarks_type remarks;

   typedef  ::hit_spider::FeetNormalVector_<ContainerAllocator>  _feetNormalVector_type;
  _feetNormalVector_type feetNormalVector;

   typedef boost::array<double, 6>  _mu_type;
  _mu_type mu;

   typedef boost::array<double, 6>  _maxNormalF_type;
  _maxNormalF_type maxNormalF;





  typedef boost::shared_ptr< ::hit_spider::hexapod_State_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hit_spider::hexapod_State_<ContainerAllocator> const> ConstPtr;

}; // struct hexapod_State_

typedef ::hit_spider::hexapod_State_<std::allocator<void> > hexapod_State;

typedef boost::shared_ptr< ::hit_spider::hexapod_State > hexapod_StatePtr;
typedef boost::shared_ptr< ::hit_spider::hexapod_State const> hexapod_StateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hit_spider::hexapod_State_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hit_spider::hexapod_State_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hit_spider::hexapod_State_<ContainerAllocator1> & lhs, const ::hit_spider::hexapod_State_<ContainerAllocator2> & rhs)
{
  return lhs.base_Pose_Now == rhs.base_Pose_Now &&
    lhs.support_State_Now == rhs.support_State_Now &&
    lhs.faultLeg_State_Now == rhs.faultLeg_State_Now &&
    lhs.feetPositionNow == rhs.feetPositionNow &&
    lhs.base_Pose_Next == rhs.base_Pose_Next &&
    lhs.support_State_Next == rhs.support_State_Next &&
    lhs.faultLeg_State_Next == rhs.faultLeg_State_Next &&
    lhs.feetPositionNext == rhs.feetPositionNext &&
    lhs.move_Direction == rhs.move_Direction &&
    lhs.remarks == rhs.remarks &&
    lhs.feetNormalVector == rhs.feetNormalVector &&
    lhs.mu == rhs.mu &&
    lhs.maxNormalF == rhs.maxNormalF;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hit_spider::hexapod_State_<ContainerAllocator1> & lhs, const ::hit_spider::hexapod_State_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hit_spider

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::hit_spider::hexapod_State_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hit_spider::hexapod_State_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hit_spider::hexapod_State_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hit_spider::hexapod_State_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hit_spider::hexapod_State_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hit_spider::hexapod_State_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hit_spider::hexapod_State_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b4dc527e58eb5ebce7216f203a48138f";
  }

  static const char* value(const ::hit_spider::hexapod_State_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb4dc527e58eb5ebcULL;
  static const uint64_t static_value2 = 0xe7216f203a48138fULL;
};

template<class ContainerAllocator>
struct DataType< ::hit_spider::hexapod_State_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hit_spider/hexapod_State";
  }

  static const char* value(const ::hit_spider::hexapod_State_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hit_spider::hexapod_State_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#当前机器人状态\n"
"hit_spider/hexapod_Base_Pose base_Pose_Now\n"
"int8[6] support_State_Now\n"
"int8[6] faultLeg_State_Now\n"
"hit_spider/FeetPosition feetPositionNow\n"
"\n"
"#下一步机器人状态\n"
"hit_spider/hexapod_Base_Pose base_Pose_Next\n"
"int8[6] support_State_Next\n"
"int8[6] faultLeg_State_Next\n"
"hit_spider/FeetPosition feetPositionNext\n"
"\n"
"#移动方向\n"
"geometry_msgs/Point move_Direction\n"
"\n"
"std_msgs/String remarks\n"
"\n"
"hit_spider/FeetNormalVector feetNormalVector\n"
"\n"
"float64[6] mu\n"
"\n"
"float64[6] maxNormalF\n"
"================================================================================\n"
"MSG: hit_spider/hexapod_Base_Pose\n"
"geometry_msgs/Point position\n"
"hit_spider/hexapod_RPY orientation\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: hit_spider/hexapod_RPY\n"
"float64 roll\n"
"float64 pitch\n"
"float64 yaw\n"
"================================================================================\n"
"MSG: hit_spider/FeetPosition\n"
"geometry_msgs/Point[6] foot\n"
"================================================================================\n"
"MSG: std_msgs/String\n"
"string data\n"
"\n"
"================================================================================\n"
"MSG: hit_spider/FeetNormalVector\n"
"geometry_msgs/Point[6] foot\n"
;
  }

  static const char* value(const ::hit_spider::hexapod_State_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hit_spider::hexapod_State_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.base_Pose_Now);
      stream.next(m.support_State_Now);
      stream.next(m.faultLeg_State_Now);
      stream.next(m.feetPositionNow);
      stream.next(m.base_Pose_Next);
      stream.next(m.support_State_Next);
      stream.next(m.faultLeg_State_Next);
      stream.next(m.feetPositionNext);
      stream.next(m.move_Direction);
      stream.next(m.remarks);
      stream.next(m.feetNormalVector);
      stream.next(m.mu);
      stream.next(m.maxNormalF);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct hexapod_State_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hit_spider::hexapod_State_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hit_spider::hexapod_State_<ContainerAllocator>& v)
  {
    s << indent << "base_Pose_Now: ";
    s << std::endl;
    Printer< ::hit_spider::hexapod_Base_Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.base_Pose_Now);
    s << indent << "support_State_Now[]" << std::endl;
    for (size_t i = 0; i < v.support_State_Now.size(); ++i)
    {
      s << indent << "  support_State_Now[" << i << "]: ";
      Printer<int8_t>::stream(s, indent + "  ", v.support_State_Now[i]);
    }
    s << indent << "faultLeg_State_Now[]" << std::endl;
    for (size_t i = 0; i < v.faultLeg_State_Now.size(); ++i)
    {
      s << indent << "  faultLeg_State_Now[" << i << "]: ";
      Printer<int8_t>::stream(s, indent + "  ", v.faultLeg_State_Now[i]);
    }
    s << indent << "feetPositionNow: ";
    s << std::endl;
    Printer< ::hit_spider::FeetPosition_<ContainerAllocator> >::stream(s, indent + "  ", v.feetPositionNow);
    s << indent << "base_Pose_Next: ";
    s << std::endl;
    Printer< ::hit_spider::hexapod_Base_Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.base_Pose_Next);
    s << indent << "support_State_Next[]" << std::endl;
    for (size_t i = 0; i < v.support_State_Next.size(); ++i)
    {
      s << indent << "  support_State_Next[" << i << "]: ";
      Printer<int8_t>::stream(s, indent + "  ", v.support_State_Next[i]);
    }
    s << indent << "faultLeg_State_Next[]" << std::endl;
    for (size_t i = 0; i < v.faultLeg_State_Next.size(); ++i)
    {
      s << indent << "  faultLeg_State_Next[" << i << "]: ";
      Printer<int8_t>::stream(s, indent + "  ", v.faultLeg_State_Next[i]);
    }
    s << indent << "feetPositionNext: ";
    s << std::endl;
    Printer< ::hit_spider::FeetPosition_<ContainerAllocator> >::stream(s, indent + "  ", v.feetPositionNext);
    s << indent << "move_Direction: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.move_Direction);
    s << indent << "remarks: ";
    s << std::endl;
    Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "  ", v.remarks);
    s << indent << "feetNormalVector: ";
    s << std::endl;
    Printer< ::hit_spider::FeetNormalVector_<ContainerAllocator> >::stream(s, indent + "  ", v.feetNormalVector);
    s << indent << "mu[]" << std::endl;
    for (size_t i = 0; i < v.mu.size(); ++i)
    {
      s << indent << "  mu[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.mu[i]);
    }
    s << indent << "maxNormalF[]" << std::endl;
    for (size_t i = 0; i < v.maxNormalF.size(); ++i)
    {
      s << indent << "  maxNormalF[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.maxNormalF[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // HIT_SPIDER_MESSAGE_HEXAPOD_STATE_H
