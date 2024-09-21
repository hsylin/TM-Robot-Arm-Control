// Generated by gencpp from file gqcnn/Action.msg
// DO NOT EDIT!


#ifndef GQCNN_MESSAGE_ACTION_H
#define GQCNN_MESSAGE_ACTION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace gqcnn
{
template <class ContainerAllocator>
struct Action_
{
  typedef Action_<ContainerAllocator> Type;

  Action_()
    : width(0)
    , height(0)
    , mask_data()
    , action_type()
    , action_data()
    , confidence(0.0)  {
    }
  Action_(const ContainerAllocator& _alloc)
    : width(0)
    , height(0)
    , mask_data(_alloc)
    , action_type(_alloc)
    , action_data(_alloc)
    , confidence(0.0)  {
  (void)_alloc;
    }



   typedef uint32_t _width_type;
  _width_type width;

   typedef uint32_t _height_type;
  _height_type height;

   typedef std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> _mask_data_type;
  _mask_data_type mask_data;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _action_type_type;
  _action_type_type action_type;

   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _action_data_type;
  _action_data_type action_data;

   typedef float _confidence_type;
  _confidence_type confidence;





  typedef boost::shared_ptr< ::gqcnn::Action_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gqcnn::Action_<ContainerAllocator> const> ConstPtr;

}; // struct Action_

typedef ::gqcnn::Action_<std::allocator<void> > Action;

typedef boost::shared_ptr< ::gqcnn::Action > ActionPtr;
typedef boost::shared_ptr< ::gqcnn::Action const> ActionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::gqcnn::Action_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::gqcnn::Action_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::gqcnn::Action_<ContainerAllocator1> & lhs, const ::gqcnn::Action_<ContainerAllocator2> & rhs)
{
  return lhs.width == rhs.width &&
    lhs.height == rhs.height &&
    lhs.mask_data == rhs.mask_data &&
    lhs.action_type == rhs.action_type &&
    lhs.action_data == rhs.action_data &&
    lhs.confidence == rhs.confidence;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::gqcnn::Action_<ContainerAllocator1> & lhs, const ::gqcnn::Action_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace gqcnn

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::gqcnn::Action_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::gqcnn::Action_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gqcnn::Action_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::gqcnn::Action_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gqcnn::Action_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::gqcnn::Action_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::gqcnn::Action_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ec928182aa4f966ba689e9ac10b10d2d";
  }

  static const char* value(const ::gqcnn::Action_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xec928182aa4f966bULL;
  static const uint64_t static_value2 = 0xa689e9ac10b10d2dULL;
};

template<class ContainerAllocator>
struct DataType< ::gqcnn::Action_<ContainerAllocator> >
{
  static const char* value()
  {
    return "gqcnn/Action";
  }

  static const char* value(const ::gqcnn::Action_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::gqcnn::Action_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Copyright ©2017. The Regents of the University of California (Regents).\n"
"# All Rights Reserved. Permission to use, copy, modify, and distribute this\n"
"# software and its documentation for educational, research, and not-for-profit\n"
"# purposes, without fee and without a signed licensing agreement, is hereby\n"
"# granted, provided that the above copyright notice, this paragraph and the\n"
"# following two paragraphs appear in all copies, modifications, and\n"
"# distributions. Contact The Office of Technology Licensing, UC Berkeley, 2150\n"
"# Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620, (510) 643-7201,\n"
"# otl@berkeley.edu,\n"
"# http://ipira.berkeley.edu/industry-info for commercial licensing opportunities.\n"
"\n"
"# IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL,\n"
"# INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF\n"
"# THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF REGENTS HAS BEEN\n"
"# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.\n"
"\n"
"# REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO,\n"
"# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR\n"
"# PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY, PROVIDED\n"
"# HEREUNDER IS PROVIDED \"AS IS\". REGENTS HAS NO OBLIGATION TO PROVIDE\n"
"# MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.\n"
"\n"
"uint32 width\n"
"uint32 height\n"
"uint8[] mask_data\n"
"string action_type\n"
"float32[] action_data\n"
"float32 confidence\n"
;
  }

  static const char* value(const ::gqcnn::Action_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::gqcnn::Action_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.width);
      stream.next(m.height);
      stream.next(m.mask_data);
      stream.next(m.action_type);
      stream.next(m.action_data);
      stream.next(m.confidence);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Action_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gqcnn::Action_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::gqcnn::Action_<ContainerAllocator>& v)
  {
    s << indent << "width: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.width);
    s << indent << "height: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.height);
    s << indent << "mask_data[]" << std::endl;
    for (size_t i = 0; i < v.mask_data.size(); ++i)
    {
      s << indent << "  mask_data[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.mask_data[i]);
    }
    s << indent << "action_type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.action_type);
    s << indent << "action_data[]" << std::endl;
    for (size_t i = 0; i < v.action_data.size(); ++i)
    {
      s << indent << "  action_data[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.action_data[i]);
    }
    s << indent << "confidence: ";
    Printer<float>::stream(s, indent + "  ", v.confidence);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GQCNN_MESSAGE_ACTION_H
