/* Auto-generated by genmsg_cpp for file /home/fredshentu/kinetic_workspace/sandbox/widomX/msg/multi_cam.msg */
#ifndef WIDOMX_MESSAGE_MULTI_CAM_H
#define WIDOMX_MESSAGE_MULTI_CAM_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/Image.h"

namespace widomX
{
template <class ContainerAllocator>
struct multi_cam_ {
  typedef multi_cam_<ContainerAllocator> Type;

  multi_cam_()
  : cam1()
  , cam2()
  {
  }

  multi_cam_(const ContainerAllocator& _alloc)
  : cam1(_alloc)
  , cam2(_alloc)
  {
  }

  typedef  ::sensor_msgs::Image_<ContainerAllocator>  _cam1_type;
   ::sensor_msgs::Image_<ContainerAllocator>  cam1;

  typedef  ::sensor_msgs::Image_<ContainerAllocator>  _cam2_type;
   ::sensor_msgs::Image_<ContainerAllocator>  cam2;


  typedef boost::shared_ptr< ::widomX::multi_cam_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::widomX::multi_cam_<ContainerAllocator>  const> ConstPtr;
}; // struct multi_cam
typedef  ::widomX::multi_cam_<std::allocator<void> > multi_cam;

typedef boost::shared_ptr< ::widomX::multi_cam> multi_camPtr;
typedef boost::shared_ptr< ::widomX::multi_cam const> multi_camConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::widomX::multi_cam_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::widomX::multi_cam_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace widomX

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::widomX::multi_cam_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::widomX::multi_cam_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::widomX::multi_cam_<ContainerAllocator> > {
  static const char* value() 
  {
    return "13989bd7259eaead1a318f01baa59910";
  }

  static const char* value(const  ::widomX::multi_cam_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x13989bd7259eaeadULL;
  static const uint64_t static_value2 = 0x1a318f01baa59910ULL;
};

template<class ContainerAllocator>
struct DataType< ::widomX::multi_cam_<ContainerAllocator> > {
  static const char* value() 
  {
    return "widomX/multi_cam";
  }

  static const char* value(const  ::widomX::multi_cam_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::widomX::multi_cam_<ContainerAllocator> > {
  static const char* value() 
  {
    return "sensor_msgs/Image cam1\n\
sensor_msgs/Image cam2\n\
\n\
================================================================================\n\
MSG: sensor_msgs/Image\n\
# This message contains an uncompressed image\n\
# (0, 0) is at top-left corner of image\n\
#\n\
\n\
Header header        # Header timestamp should be acquisition time of image\n\
                     # Header frame_id should be optical frame of camera\n\
                     # origin of frame should be optical center of cameara\n\
                     # +x should point to the right in the image\n\
                     # +y should point down in the image\n\
                     # +z should point into to plane of the image\n\
                     # If the frame_id here and the frame_id of the CameraInfo\n\
                     # message associated with the image conflict\n\
                     # the behavior is undefined\n\
\n\
uint32 height         # image height, that is, number of rows\n\
uint32 width          # image width, that is, number of columns\n\
\n\
# The legal values for encoding are in file src/image_encodings.cpp\n\
# If you want to standardize a new string format, join\n\
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n\
\n\
string encoding       # Encoding of pixels -- channel meaning, ordering, size\n\
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n\
\n\
uint8 is_bigendian    # is this data bigendian?\n\
uint32 step           # Full row length in bytes\n\
uint8[] data          # actual matrix data, size is (step * rows)\n\
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
";
  }

  static const char* value(const  ::widomX::multi_cam_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::widomX::multi_cam_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.cam1);
    stream.next(m.cam2);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct multi_cam_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::widomX::multi_cam_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::widomX::multi_cam_<ContainerAllocator> & v) 
  {
    s << indent << "cam1: ";
s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.cam1);
    s << indent << "cam2: ";
s << std::endl;
    Printer< ::sensor_msgs::Image_<ContainerAllocator> >::stream(s, indent + "  ", v.cam2);
  }
};


} // namespace message_operations
} // namespace ros

#endif // WIDOMX_MESSAGE_MULTI_CAM_H

