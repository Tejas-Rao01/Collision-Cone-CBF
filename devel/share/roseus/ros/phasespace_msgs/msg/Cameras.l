;; Auto-generated. Do not edit!


(when (boundp 'phasespace_msgs::Cameras)
  (if (not (find-package "PHASESPACE_MSGS"))
    (make-package "PHASESPACE_MSGS"))
  (shadow 'Cameras (find-package "PHASESPACE_MSGS")))
(unless (find-package "PHASESPACE_MSGS::CAMERAS")
  (make-package "PHASESPACE_MSGS::CAMERAS"))

(in-package "ROS")
;;//! \htmlinclude Cameras.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass phasespace_msgs::Cameras
  :super ros::object
  :slots (_header _cameras ))

(defmethod phasespace_msgs::Cameras
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:cameras __cameras) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _cameras __cameras)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:cameras
   (&rest __cameras)
   (if (keywordp (car __cameras))
       (send* _cameras __cameras)
     (progn
       (if __cameras (setq _cameras (car __cameras)))
       _cameras)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; phasespace_msgs/Camera[] _cameras
    (apply #'+ (send-all _cameras :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; phasespace_msgs/Camera[] _cameras
     (write-long (length _cameras) s)
     (dolist (elem _cameras)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; phasespace_msgs/Camera[] _cameras
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cameras (let (r) (dotimes (i n) (push (instance phasespace_msgs::Camera :init) r)) r))
     (dolist (elem- _cameras)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get phasespace_msgs::Cameras :md5sum-) "5f4b3dddb1243eb3e913cbf3e1940fe8")
(setf (get phasespace_msgs::Cameras :datatype-) "phasespace_msgs/Cameras")
(setf (get phasespace_msgs::Cameras :definition-)
      "########################################
# Messages
########################################
std_msgs/Header header
Camera[] cameras
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: phasespace_msgs/Camera
########################################
# Messages
########################################
uint32 id
uint32 flags
float32 x
float32 y
float32 z
float32 qw
float32 qx
float32 qy
float32 qz
float32 cond
")



(provide :phasespace_msgs/Cameras "5f4b3dddb1243eb3e913cbf3e1940fe8")


