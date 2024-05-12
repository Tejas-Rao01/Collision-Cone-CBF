; Auto-generated. Do not edit!


(cl:in-package phasespace_msgs-msg)


;//! \htmlinclude Camera.msg.html

(cl:defclass <Camera> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (flags
    :reader flags
    :initarg :flags
    :type cl:integer
    :initform 0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (qw
    :reader qw
    :initarg :qw
    :type cl:float
    :initform 0.0)
   (qx
    :reader qx
    :initarg :qx
    :type cl:float
    :initform 0.0)
   (qy
    :reader qy
    :initarg :qy
    :type cl:float
    :initform 0.0)
   (qz
    :reader qz
    :initarg :qz
    :type cl:float
    :initform 0.0)
   (cond
    :reader cond
    :initarg :cond
    :type cl:float
    :initform 0.0))
)

(cl:defclass Camera (<Camera>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Camera>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Camera)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phasespace_msgs-msg:<Camera> is deprecated: use phasespace_msgs-msg:Camera instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Camera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phasespace_msgs-msg:id-val is deprecated.  Use phasespace_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'flags-val :lambda-list '(m))
(cl:defmethod flags-val ((m <Camera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phasespace_msgs-msg:flags-val is deprecated.  Use phasespace_msgs-msg:flags instead.")
  (flags m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Camera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phasespace_msgs-msg:x-val is deprecated.  Use phasespace_msgs-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Camera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phasespace_msgs-msg:y-val is deprecated.  Use phasespace_msgs-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <Camera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phasespace_msgs-msg:z-val is deprecated.  Use phasespace_msgs-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'qw-val :lambda-list '(m))
(cl:defmethod qw-val ((m <Camera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phasespace_msgs-msg:qw-val is deprecated.  Use phasespace_msgs-msg:qw instead.")
  (qw m))

(cl:ensure-generic-function 'qx-val :lambda-list '(m))
(cl:defmethod qx-val ((m <Camera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phasespace_msgs-msg:qx-val is deprecated.  Use phasespace_msgs-msg:qx instead.")
  (qx m))

(cl:ensure-generic-function 'qy-val :lambda-list '(m))
(cl:defmethod qy-val ((m <Camera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phasespace_msgs-msg:qy-val is deprecated.  Use phasespace_msgs-msg:qy instead.")
  (qy m))

(cl:ensure-generic-function 'qz-val :lambda-list '(m))
(cl:defmethod qz-val ((m <Camera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phasespace_msgs-msg:qz-val is deprecated.  Use phasespace_msgs-msg:qz instead.")
  (qz m))

(cl:ensure-generic-function 'cond-val :lambda-list '(m))
(cl:defmethod cond-val ((m <Camera>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phasespace_msgs-msg:cond-val is deprecated.  Use phasespace_msgs-msg:cond instead.")
  (cond m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Camera>) ostream)
  "Serializes a message object of type '<Camera>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'qw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'qx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'qy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'qz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cond))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Camera>) istream)
  "Deserializes a message object of type '<Camera>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'qw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'qx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'qy) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'qz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cond) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Camera>)))
  "Returns string type for a message object of type '<Camera>"
  "phasespace_msgs/Camera")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Camera)))
  "Returns string type for a message object of type 'Camera"
  "phasespace_msgs/Camera")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Camera>)))
  "Returns md5sum for a message object of type '<Camera>"
  "94c328fde1ac5baf2cadea9ffd749094")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Camera)))
  "Returns md5sum for a message object of type 'Camera"
  "94c328fde1ac5baf2cadea9ffd749094")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Camera>)))
  "Returns full string definition for message of type '<Camera>"
  (cl:format cl:nil "########################################~%# Messages~%########################################~%uint32 id~%uint32 flags~%float32 x~%float32 y~%float32 z~%float32 qw~%float32 qx~%float32 qy~%float32 qz~%float32 cond~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Camera)))
  "Returns full string definition for message of type 'Camera"
  (cl:format cl:nil "########################################~%# Messages~%########################################~%uint32 id~%uint32 flags~%float32 x~%float32 y~%float32 z~%float32 qw~%float32 qx~%float32 qy~%float32 qz~%float32 cond~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Camera>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Camera>))
  "Converts a ROS message object to a list"
  (cl:list 'Camera
    (cl:cons ':id (id msg))
    (cl:cons ':flags (flags msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':qw (qw msg))
    (cl:cons ':qx (qx msg))
    (cl:cons ':qy (qy msg))
    (cl:cons ':qz (qz msg))
    (cl:cons ':cond (cond msg))
))
