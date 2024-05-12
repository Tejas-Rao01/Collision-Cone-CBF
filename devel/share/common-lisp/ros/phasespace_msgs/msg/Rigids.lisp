; Auto-generated. Do not edit!


(cl:in-package phasespace_msgs-msg)


;//! \htmlinclude Rigids.msg.html

(cl:defclass <Rigids> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (rigids
    :reader rigids
    :initarg :rigids
    :type (cl:vector phasespace_msgs-msg:Rigid)
   :initform (cl:make-array 0 :element-type 'phasespace_msgs-msg:Rigid :initial-element (cl:make-instance 'phasespace_msgs-msg:Rigid))))
)

(cl:defclass Rigids (<Rigids>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Rigids>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Rigids)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phasespace_msgs-msg:<Rigids> is deprecated: use phasespace_msgs-msg:Rigids instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Rigids>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phasespace_msgs-msg:header-val is deprecated.  Use phasespace_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'rigids-val :lambda-list '(m))
(cl:defmethod rigids-val ((m <Rigids>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phasespace_msgs-msg:rigids-val is deprecated.  Use phasespace_msgs-msg:rigids instead.")
  (rigids m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Rigids>) ostream)
  "Serializes a message object of type '<Rigids>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rigids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'rigids))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Rigids>) istream)
  "Deserializes a message object of type '<Rigids>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rigids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'rigids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'phasespace_msgs-msg:Rigid))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Rigids>)))
  "Returns string type for a message object of type '<Rigids>"
  "phasespace_msgs/Rigids")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Rigids)))
  "Returns string type for a message object of type 'Rigids"
  "phasespace_msgs/Rigids")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Rigids>)))
  "Returns md5sum for a message object of type '<Rigids>"
  "1db17ace582e60bc2646dffe29b31add")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Rigids)))
  "Returns md5sum for a message object of type 'Rigids"
  "1db17ace582e60bc2646dffe29b31add")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Rigids>)))
  "Returns full string definition for message of type '<Rigids>"
  (cl:format cl:nil "########################################~%# Messages~%########################################~%std_msgs/Header header~%Rigid[] rigids~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: phasespace_msgs/Rigid~%########################################~%# Messages~%########################################~%uint32 id~%uint32 flags~%uint64 time~%float32 x~%float32 y~%float32 z~%float32 qw~%float32 qx~%float32 qy~%float32 qz~%float32 cond~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Rigids)))
  "Returns full string definition for message of type 'Rigids"
  (cl:format cl:nil "########################################~%# Messages~%########################################~%std_msgs/Header header~%Rigid[] rigids~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: phasespace_msgs/Rigid~%########################################~%# Messages~%########################################~%uint32 id~%uint32 flags~%uint64 time~%float32 x~%float32 y~%float32 z~%float32 qw~%float32 qx~%float32 qy~%float32 qz~%float32 cond~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Rigids>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rigids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Rigids>))
  "Converts a ROS message object to a list"
  (cl:list 'Rigids
    (cl:cons ':header (header msg))
    (cl:cons ':rigids (rigids msg))
))
