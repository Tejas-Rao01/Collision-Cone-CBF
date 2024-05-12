;; Auto-generated. Do not edit!


(when (boundp 'phasespace_msgs::Marker)
  (if (not (find-package "PHASESPACE_MSGS"))
    (make-package "PHASESPACE_MSGS"))
  (shadow 'Marker (find-package "PHASESPACE_MSGS")))
(unless (find-package "PHASESPACE_MSGS::MARKER")
  (make-package "PHASESPACE_MSGS::MARKER"))

(in-package "ROS")
;;//! \htmlinclude Marker.msg.html


(defclass phasespace_msgs::Marker
  :super ros::object
  :slots (_id _flags _time _x _y _z _cond ))

(defmethod phasespace_msgs::Marker
  (:init
   (&key
    ((:id __id) 0)
    ((:flags __flags) 0)
    ((:time __time) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:cond __cond) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _flags (round __flags))
   (setq _time (round __time))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _cond (float __cond))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:flags
   (&optional __flags)
   (if __flags (setq _flags __flags)) _flags)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
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
    ;; uint64 _time
    8
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
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
     ;; uint64 _time
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _time (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _time) (= (length (_time . bv)) 2)) ;; bignum
              (write-long (ash (elt (_time . bv) 0) 0) s)
              (write-long (ash (elt (_time . bv) 1) -1) s))
             ((and (class _time) (= (length (_time . bv)) 1)) ;; big1
              (write-long (elt (_time . bv) 0) s)
              (write-long (if (>= _time 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _time s)(write-long (if (>= _time 0) 0 #xffffffff) s)))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; uint64 _time
     
#+(or :alpha :irix6 :x86_64)
      (setf _time (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _time (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cond
     (setq _cond (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get phasespace_msgs::Marker :md5sum-) "263d5635c15e661dd0615ba6f237c9a9")
(setf (get phasespace_msgs::Marker :datatype-) "phasespace_msgs/Marker")
(setf (get phasespace_msgs::Marker :definition-)
      "########################################
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



(provide :phasespace_msgs/Marker "263d5635c15e661dd0615ba6f237c9a9")


