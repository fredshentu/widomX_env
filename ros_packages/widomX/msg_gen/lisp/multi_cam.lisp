; Auto-generated. Do not edit!


(cl:in-package widomX-msg)


;//! \htmlinclude multi_cam.msg.html

(cl:defclass <multi_cam> (roslisp-msg-protocol:ros-message)
  ((cam1
    :reader cam1
    :initarg :cam1
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (cam2
    :reader cam2
    :initarg :cam2
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass multi_cam (<multi_cam>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <multi_cam>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'multi_cam)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name widomX-msg:<multi_cam> is deprecated: use widomX-msg:multi_cam instead.")))

(cl:ensure-generic-function 'cam1-val :lambda-list '(m))
(cl:defmethod cam1-val ((m <multi_cam>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader widomX-msg:cam1-val is deprecated.  Use widomX-msg:cam1 instead.")
  (cam1 m))

(cl:ensure-generic-function 'cam2-val :lambda-list '(m))
(cl:defmethod cam2-val ((m <multi_cam>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader widomX-msg:cam2-val is deprecated.  Use widomX-msg:cam2 instead.")
  (cam2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <multi_cam>) ostream)
  "Serializes a message object of type '<multi_cam>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cam1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cam2) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <multi_cam>) istream)
  "Deserializes a message object of type '<multi_cam>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cam1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cam2) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<multi_cam>)))
  "Returns string type for a message object of type '<multi_cam>"
  "widomX/multi_cam")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'multi_cam)))
  "Returns string type for a message object of type 'multi_cam"
  "widomX/multi_cam")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<multi_cam>)))
  "Returns md5sum for a message object of type '<multi_cam>"
  "13989bd7259eaead1a318f01baa59910")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'multi_cam)))
  "Returns md5sum for a message object of type 'multi_cam"
  "13989bd7259eaead1a318f01baa59910")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<multi_cam>)))
  "Returns full string definition for message of type '<multi_cam>"
  (cl:format cl:nil "sensor_msgs/Image cam1~%sensor_msgs/Image cam2~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'multi_cam)))
  "Returns full string definition for message of type 'multi_cam"
  (cl:format cl:nil "sensor_msgs/Image cam1~%sensor_msgs/Image cam2~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <multi_cam>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cam1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cam2))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <multi_cam>))
  "Converts a ROS message object to a list"
  (cl:list 'multi_cam
    (cl:cons ':cam1 (cam1 msg))
    (cl:cons ':cam2 (cam2 msg))
))
