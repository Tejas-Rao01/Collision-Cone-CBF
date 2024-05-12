;; Auto-generated. Do not edit!


(when (boundp 'phasespace_msgs::Markers)
  (if (not (find-package "PHASESPACE_MSGS"))
    (make-package "PHASESPACE_MSGS"))
  (shadow 'Markers (find-package "PHASESPACE_MSGS")))
(unless (find-package "PHASESPACE_MSGS::MARKERS")
  (make-package "PHASESPACE_MSGS::MARKERS"))

(in-package "ROS")
;;//! \htmlinclude Markers.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass phasespace_msgs::Markers
  :super ros::object
  :slots (_header _markers ))

(defmethod phasespace_msgs::Markers
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:markers __markers) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _markers __markers)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:markers
   (&rest __markers)
   (if (keywordp (car __markers))
       (send* _markers __markers)
     (progn
       (if __markers (setq _markers (car __markers)))
       _markers)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; phasespace_msgs/Marker[] _markers
    (apply #'+ (send-all _markers :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; phasespace_msgs/Marker[] _markers
     (write-long (length _markers) s)
     (dolist (elem _markers)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; phasespace_msgs/Marker[] _markers
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _markers (let (r) (dotimes (i n) (push (instance phasespace_msgs::Marker :init) r)) r))
     (dolist (elem- _markers)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get phasespace_msgs::Markers :md5sum-) "87fe27df7445224826bbc3363f7dd056")
(setf (get phasespace_msgs::Markers :datatype-) "phasespace_msgs/Markers")
(setf (get phasespace_msgs::Markers :definition-)
      "########################################
# Messages
########################################
std_msgs/Header header
Marker[] markers
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
MSG: phasespace_msgs/Marker
########################################
# Messages
########################################
uint32 id
uint32 flags
uint64 time
float32 x
float32 y
float32 z
float32 cond
")



(provide :phasespace_msgs/Markers "87fe27df7445224826bbc3363f7dd056")


