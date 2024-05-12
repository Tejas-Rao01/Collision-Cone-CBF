; Auto-generated. Do not edit!


(cl:in-package phasespace_msgs-msg)


;//! \htmlinclude Cameras.msg.html

(cl:defclass <Cameras> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (cameras
    :reader cameras
    :initarg :cameras
    :type (cl:vector phasespace_msgs-msg:Camera)
   :initform (cl:make-array 0 :element-type 'phasespace_msgs-msg:Camera :initial-element (cl:make-instance 'phasespace_msgs-msg:Camera))))
)

(cl:defclass Cameras (<Cameras>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Cameras>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Cameras)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phasespace_msgs-msg:<Cameras> is deprecated: use phasespace_msgs-msg:Cameras instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Cameras>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phasespace_msgs-msg:header-val is deprecated.  Use phasespace_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'cameras-val :lambda-list '(m))
(cl:defmethod cameras-val ((m <Cameras>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phasespace_msgs-msg:cameras-val is deprecated.  Use phasespace_msgs-msg:cameras instead.")
  (cameras m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Cameras>) ostream)
  "Serializes a message object of type '<Cameras>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cameras))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cameras))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Cameras>) istream)
  "Deserializes a message object of type '<Cameras>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cameras) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cameras)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'phasespace_msgs-msg:Camera))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Cameras>)))
  "Returns string type for a message object of type '<Cameras>"
  "phasespace_msgs/Cameras")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Cameras)))
  "Returns string type for a message object of type 'Cameras"
  "phasespace_msgs/Cameras")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Cameras>)))
  "Returns md5sum for a message object of type '<Cameras>"
  "5f4b3dddb1243eb3e913cbf3e1940fe8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Cameras)))
  "Returns md5sum for a message object of type 'Cameras"
  "5f4b3dddb1243eb3e913cbf3e1940fe8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Cameras>)))
  "Returns full string definition for message of type '<Cameras>"
  (cl:format cl:nil "########################################~%# Messages~%########################################~%std_msgs/Header header~%Camera[] cameras~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: phasespace_msgs/Camera~%########################################~%# Messages~%########################################~%uint32 id~%uint32 flags~%float32 x~%float32 y~%float32 z~%float32 qw~%float32 qx~%float32 qy~%float32 qz~%float32 cond~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Cameras)))
  "Returns full string definition for message of type 'Cameras"
  (cl:format cl:nil "########################################~%# Messages~%########################################~%std_msgs/Header header~%Camera[] cameras~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: phasespace_msgs/Camera~%########################################~%# Messages~%########################################~%uint32 id~%uint32 flags~%float32 x~%float32 y~%float32 z~%float32 qw~%float32 qx~%float32 qy~%float32 qz~%float32 cond~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Cameras>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cameras) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Cameras>))
  "Converts a ROS message object to a list"
  (cl:list 'Cameras
    (cl:cons ':header (header msg))
    (cl:cons ':cameras (cameras msg))
))
