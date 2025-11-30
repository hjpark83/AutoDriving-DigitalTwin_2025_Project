;; Auto-generated. Do not edit!


(when (boundp 'dcas_msgs::TrafficSign)
  (if (not (find-package "DCAS_MSGS"))
    (make-package "DCAS_MSGS"))
  (shadow 'TrafficSign (find-package "DCAS_MSGS")))
(unless (find-package "DCAS_MSGS::TRAFFICSIGN")
  (make-package "DCAS_MSGS::TRAFFICSIGN"))

(in-package "ROS")
;;//! \htmlinclude TrafficSign.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*CLASS_UNKNOWN*" (find-package "DCAS_MSGS::TRAFFICSIGN"))
(shadow '*CLASS_UNKNOWN* (find-package "DCAS_MSGS::TRAFFICSIGN"))
(defconstant dcas_msgs::TrafficSign::*CLASS_UNKNOWN* 0)
(intern "*CLASS_STOP*" (find-package "DCAS_MSGS::TRAFFICSIGN"))
(shadow '*CLASS_STOP* (find-package "DCAS_MSGS::TRAFFICSIGN"))
(defconstant dcas_msgs::TrafficSign::*CLASS_STOP* 1)
(intern "*CLASS_YIELD*" (find-package "DCAS_MSGS::TRAFFICSIGN"))
(shadow '*CLASS_YIELD* (find-package "DCAS_MSGS::TRAFFICSIGN"))
(defconstant dcas_msgs::TrafficSign::*CLASS_YIELD* 2)
(intern "*CLASS_SPEED_LIMIT_30*" (find-package "DCAS_MSGS::TRAFFICSIGN"))
(shadow '*CLASS_SPEED_LIMIT_30* (find-package "DCAS_MSGS::TRAFFICSIGN"))
(defconstant dcas_msgs::TrafficSign::*CLASS_SPEED_LIMIT_30* 3)
(intern "*CLASS_SPEED_LIMIT_50*" (find-package "DCAS_MSGS::TRAFFICSIGN"))
(shadow '*CLASS_SPEED_LIMIT_50* (find-package "DCAS_MSGS::TRAFFICSIGN"))
(defconstant dcas_msgs::TrafficSign::*CLASS_SPEED_LIMIT_50* 4)
(intern "*CLASS_SPEED_LIMIT_60*" (find-package "DCAS_MSGS::TRAFFICSIGN"))
(shadow '*CLASS_SPEED_LIMIT_60* (find-package "DCAS_MSGS::TRAFFICSIGN"))
(defconstant dcas_msgs::TrafficSign::*CLASS_SPEED_LIMIT_60* 5)
(intern "*CLASS_SPEED_LIMIT_80*" (find-package "DCAS_MSGS::TRAFFICSIGN"))
(shadow '*CLASS_SPEED_LIMIT_80* (find-package "DCAS_MSGS::TRAFFICSIGN"))
(defconstant dcas_msgs::TrafficSign::*CLASS_SPEED_LIMIT_80* 6)
(intern "*CLASS_NO_ENTRY*" (find-package "DCAS_MSGS::TRAFFICSIGN"))
(shadow '*CLASS_NO_ENTRY* (find-package "DCAS_MSGS::TRAFFICSIGN"))
(defconstant dcas_msgs::TrafficSign::*CLASS_NO_ENTRY* 7)
(intern "*CLASS_NO_PARKING*" (find-package "DCAS_MSGS::TRAFFICSIGN"))
(shadow '*CLASS_NO_PARKING* (find-package "DCAS_MSGS::TRAFFICSIGN"))
(defconstant dcas_msgs::TrafficSign::*CLASS_NO_PARKING* 8)
(intern "*CLASS_PEDESTRIAN_CROSSING*" (find-package "DCAS_MSGS::TRAFFICSIGN"))
(shadow '*CLASS_PEDESTRIAN_CROSSING* (find-package "DCAS_MSGS::TRAFFICSIGN"))
(defconstant dcas_msgs::TrafficSign::*CLASS_PEDESTRIAN_CROSSING* 9)
(intern "*CLASS_SCHOOL_ZONE*" (find-package "DCAS_MSGS::TRAFFICSIGN"))
(shadow '*CLASS_SCHOOL_ZONE* (find-package "DCAS_MSGS::TRAFFICSIGN"))
(defconstant dcas_msgs::TrafficSign::*CLASS_SCHOOL_ZONE* 10)
(intern "*CLASS_CONSTRUCTION*" (find-package "DCAS_MSGS::TRAFFICSIGN"))
(shadow '*CLASS_CONSTRUCTION* (find-package "DCAS_MSGS::TRAFFICSIGN"))
(defconstant dcas_msgs::TrafficSign::*CLASS_CONSTRUCTION* 11)
(intern "*CLASS_TURN_LEFT*" (find-package "DCAS_MSGS::TRAFFICSIGN"))
(shadow '*CLASS_TURN_LEFT* (find-package "DCAS_MSGS::TRAFFICSIGN"))
(defconstant dcas_msgs::TrafficSign::*CLASS_TURN_LEFT* 12)
(intern "*CLASS_TURN_RIGHT*" (find-package "DCAS_MSGS::TRAFFICSIGN"))
(shadow '*CLASS_TURN_RIGHT* (find-package "DCAS_MSGS::TRAFFICSIGN"))
(defconstant dcas_msgs::TrafficSign::*CLASS_TURN_RIGHT* 13)
(intern "*CLASS_GO_STRAIGHT*" (find-package "DCAS_MSGS::TRAFFICSIGN"))
(shadow '*CLASS_GO_STRAIGHT* (find-package "DCAS_MSGS::TRAFFICSIGN"))
(defconstant dcas_msgs::TrafficSign::*CLASS_GO_STRAIGHT* 14)

(defun dcas_msgs::TrafficSign-to-symbol (const)
  (cond
        ((= const 0) 'dcas_msgs::TrafficSign::*CLASS_UNKNOWN*)
        ((= const 1) 'dcas_msgs::TrafficSign::*CLASS_STOP*)
        ((= const 2) 'dcas_msgs::TrafficSign::*CLASS_YIELD*)
        ((= const 3) 'dcas_msgs::TrafficSign::*CLASS_SPEED_LIMIT_30*)
        ((= const 4) 'dcas_msgs::TrafficSign::*CLASS_SPEED_LIMIT_50*)
        ((= const 5) 'dcas_msgs::TrafficSign::*CLASS_SPEED_LIMIT_60*)
        ((= const 6) 'dcas_msgs::TrafficSign::*CLASS_SPEED_LIMIT_80*)
        ((= const 7) 'dcas_msgs::TrafficSign::*CLASS_NO_ENTRY*)
        ((= const 8) 'dcas_msgs::TrafficSign::*CLASS_NO_PARKING*)
        ((= const 9) 'dcas_msgs::TrafficSign::*CLASS_PEDESTRIAN_CROSSING*)
        ((= const 10) 'dcas_msgs::TrafficSign::*CLASS_SCHOOL_ZONE*)
        ((= const 11) 'dcas_msgs::TrafficSign::*CLASS_CONSTRUCTION*)
        ((= const 12) 'dcas_msgs::TrafficSign::*CLASS_TURN_LEFT*)
        ((= const 13) 'dcas_msgs::TrafficSign::*CLASS_TURN_RIGHT*)
        ((= const 14) 'dcas_msgs::TrafficSign::*CLASS_GO_STRAIGHT*)
        (t nil)))

(defclass dcas_msgs::TrafficSign
  :super ros::object
  :slots (_header _id _pose _sign_class ))

(defmethod dcas_msgs::TrafficSign
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id __id) 0)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:sign_class __sign_class) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _id (round __id))
   (setq _pose __pose)
   (setq _sign_class (round __sign_class))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:sign_class
   (&optional __sign_class)
   (if __sign_class (setq _sign_class __sign_class)) _sign_class)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _id
    4
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; uint8 _sign_class
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _id
       (write-long _id s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; uint8 _sign_class
       (write-byte _sign_class s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; uint8 _sign_class
     (setq _sign_class (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get dcas_msgs::TrafficSign :md5sum-) "c95b6f353f8d2898545192a2a0de59a6")
(setf (get dcas_msgs::TrafficSign :datatype-) "dcas_msgs/TrafficSign")
(setf (get dcas_msgs::TrafficSign :definition-)
      "std_msgs/Header header
uint32 id
geometry_msgs/Pose pose
uint8 sign_class

# Traffic sign classes
uint8 CLASS_UNKNOWN=0
uint8 CLASS_STOP=1
uint8 CLASS_YIELD=2
uint8 CLASS_SPEED_LIMIT_30=3
uint8 CLASS_SPEED_LIMIT_50=4
uint8 CLASS_SPEED_LIMIT_60=5
uint8 CLASS_SPEED_LIMIT_80=6
uint8 CLASS_NO_ENTRY=7
uint8 CLASS_NO_PARKING=8
uint8 CLASS_PEDESTRIAN_CROSSING=9
uint8 CLASS_SCHOOL_ZONE=10
uint8 CLASS_CONSTRUCTION=11
uint8 CLASS_TURN_LEFT=12
uint8 CLASS_TURN_RIGHT=13
uint8 CLASS_GO_STRAIGHT=14



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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :dcas_msgs/TrafficSign "c95b6f353f8d2898545192a2a0de59a6")


