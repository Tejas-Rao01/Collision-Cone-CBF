;; Auto-generated. Do not edit!


(when (boundp 'phasespace_msgs::Rigids)
  (if (not (find-package "PHASESPACE_MSGS"))
    (make-package "PHASESPACE_MSGS"))
  (shadow 'Rigids (find-package "PHASESPACE_MSGS")))
(unless (find-package "PHASESPACE_MSGS::RIGIDS")
  (make-package "PHASESPACE_MSGS::RIGIDS"))

(in-package "ROS")
;;//! \htmlinclude Rigids.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass phasespace_msgs::Rigids
  :super ros::object
  :slots (_header _rigids ))

(defmethod phasespace_msgs::Rigids
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:rigids __rigids) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _rigids __rigids)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:rigids
   (&rest __rigids)
   (if (keywordp (car __rigids))
       (send* _rigids __rigids)
     (progn
       (if __rigids (setq _rigids (car __rigids)))
       _rigids)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; phasespace_msgs/Rigid[] _rigids
    (apply #'+ (send-all _rigids :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; phasespace_msgs/Rigid[] _rigids
     (write-long (length _rigids) s)
     (dolist (elem _rigids)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; phasespace_msgs/Rigid[] _rigids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rigids (let (r) (dotimes (i n) (push (instance phasespace_msgs::Rigid :init) r)) r))
     (dolist (elem- _rigids)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get phasespace_msgs::Rigids :md5sum-) "1db17ace582e60bc2646dffe29b31add")
(setf (get phasespace_msgs::Rigids :datatype-) "phasespace_msgs/Rigids")
(setf (get phasespace_msgs::Rigids :definition-)
      "########################################
# Messages
########################################
std_msgs/Header header
Rigid[] rigids
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
MSG: phasespace_msgs/Rigid
########################################
# Messages
########################################
uint32 id
uint32 flags
uint64 time
float32 x
float32 y
float32 z
float32 qw
float32 qx
float32 qy
float32 qz
float32 cond
")



(provide :phasespace_msgs/Rigids "1db17ace582e60bc2646dffe29b31add")


