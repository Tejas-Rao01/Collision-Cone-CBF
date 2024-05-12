;; Auto-generated. Do not edit!


(when (boundp 'phasespace_msgs::Camera)
  (if (not (find-package "PHASESPACE_MSGS"))
    (make-package "PHASESPACE_MSGS"))
  (shadow 'Camera (find-package "PHASESPACE_MSGS")))
(unless (find-package "PHASESPACE_MSGS::CAMERA")
  (make-package "PHASESPACE_MSGS::CAMERA"))

(in-package "ROS")
;;//! \htmlinclude Camera.msg.html


(defclass phasespace_msgs::Camera
  :super ros::object
  :slots (_id _flags _x _y _z _qw _qx _qy _qz _cond ))

(defmethod phasespace_msgs::Camera
  (:init
   (&key
    ((:id __id) 0)
    ((:flags __flags) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:qw __qw) 0.0)
    ((:qx __qx) 0.0)
    ((:qy __qy) 0.0)
    ((:qz __qz) 0.0)
    ((:cond __cond) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _flags (round __flags))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _qw (float __qw))
   (setq _qx (float __qx))
   (setq _qy (float __qy))
   (setq _qz (float __qz))
   (setq _cond (float __cond))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:flags
   (&optional __flags)
   (if __flags (setq _flags __flags)) _flags)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:qw
   (&optional __qw)
   (if __qw (setq _qw __qw)) _qw)
  (:qx
   (&optional __qx)
   (if __qx (setq _qx __qx)) _qx)
  (:qy
   (&optional __qy)
   (if __qy (setq _qy __qy)) _qy)
  (:qz
   (&optional __qz)
   (if __qz (setq _qz __qz)) _qz)
  (:cond
   (&optional __cond)
   (if __cond (setq _cond __cond)) _cond)
  (:serialization-length
   ()
   (+
    ;; uint32 _id
    4
    ;; uint32 _flags
    4
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ;; float32 _qw
    4
    ;; float32 _qx
    4
    ;; float32 _qy
    4
    ;; float32 _qz
    4
    ;; float32 _cond
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _id
       (write-long _id s)
     ;; uint32 _flags
       (write-long _flags s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _qw
       (sys::poke _qw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _qx
       (sys::poke _qx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _qy
       (sys::poke _qy (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _qz
       (sys::poke _qz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cond
       (sys::poke _cond (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _flags
     (setq _flags (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _qw
     (setq _qw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _qx
     (setq _qx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _qy
     (setq _qy (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _qz
     (setq _qz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cond
     (setq _cond (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get phasespace_msgs::Camera :md5sum-) "94c328fde1ac5baf2cadea9ffd749094")
(setf (get phasespace_msgs::Camera :datatype-) "phasespace_msgs/Camera")
(setf (get phasespace_msgs::Camera :definition-)
      "########################################
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



(provide :phasespace_msgs/Camera "94c328fde1ac5baf2cadea9ffd749094")


