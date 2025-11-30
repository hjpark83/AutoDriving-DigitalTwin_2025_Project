;; Auto-generated. Do not edit!


(when (boundp 'dcas_msgs::TrafficLightArray)
  (if (not (find-package "DCAS_MSGS"))
    (make-package "DCAS_MSGS"))
  (shadow 'TrafficLightArray (find-package "DCAS_MSGS")))
(unless (find-package "DCAS_MSGS::TRAFFICLIGHTARRAY")
  (make-package "DCAS_MSGS::TRAFFICLIGHTARRAY"))

(in-package "ROS")
;;//! \htmlinclude TrafficLightArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass dcas_msgs::TrafficLightArray
  :super ros::object
  :slots (_header _lights ))

(defmethod dcas_msgs::TrafficLightArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lights __lights) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _lights __lights)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lights
   (&rest __lights)
   (if (keywordp (car __lights))
       (send* _lights __lights)
     (progn
       (if __lights (setq _lights (car __lights)))
       _lights)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; dcas_msgs/TrafficLight[] _lights
    (apply #'+ (send-all _lights :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; dcas_msgs/TrafficLight[] _lights
     (write-long (length _lights) s)
     (dolist (elem _lights)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; dcas_msgs/TrafficLight[] _lights
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _lights (let (r) (dotimes (i n) (push (instance dcas_msgs::TrafficLight :init) r)) r))
     (dolist (elem- _lights)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get dcas_msgs::TrafficLightArray :md5sum-) "ae3faa55d431d7df7116cd9eb141ea12")
(setf (get dcas_msgs::TrafficLightArray :datatype-) "dcas_msgs/TrafficLightArray")
(setf (get dcas_msgs::TrafficLightArray :definition-)
      "std_msgs/Header header
dcas_msgs/TrafficLight[] lights



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
MSG: dcas_msgs/TrafficLight
std_msgs/Header header
geometry_msgs/Pose pose
uint8 state

uint8 RED=0
uint8 YELLOW=1
uint8 GREEN=2



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



(provide :dcas_msgs/TrafficLightArray "ae3faa55d431d7df7116cd9eb141ea12")


