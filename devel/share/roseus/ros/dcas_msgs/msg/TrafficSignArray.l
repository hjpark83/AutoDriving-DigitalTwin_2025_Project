;; Auto-generated. Do not edit!


(when (boundp 'dcas_msgs::TrafficSignArray)
  (if (not (find-package "DCAS_MSGS"))
    (make-package "DCAS_MSGS"))
  (shadow 'TrafficSignArray (find-package "DCAS_MSGS")))
(unless (find-package "DCAS_MSGS::TRAFFICSIGNARRAY")
  (make-package "DCAS_MSGS::TRAFFICSIGNARRAY"))

(in-package "ROS")
;;//! \htmlinclude TrafficSignArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass dcas_msgs::TrafficSignArray
  :super ros::object
  :slots (_header _signs ))

(defmethod dcas_msgs::TrafficSignArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:signs __signs) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _signs __signs)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:signs
   (&rest __signs)
   (if (keywordp (car __signs))
       (send* _signs __signs)
     (progn
       (if __signs (setq _signs (car __signs)))
       _signs)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; dcas_msgs/TrafficSign[] _signs
    (apply #'+ (send-all _signs :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; dcas_msgs/TrafficSign[] _signs
     (write-long (length _signs) s)
     (dolist (elem _signs)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; dcas_msgs/TrafficSign[] _signs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _signs (let (r) (dotimes (i n) (push (instance dcas_msgs::TrafficSign :init) r)) r))
     (dolist (elem- _signs)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get dcas_msgs::TrafficSignArray :md5sum-) "06d9b79e9aad276f858f3ff080587c81")
(setf (get dcas_msgs::TrafficSignArray :datatype-) "dcas_msgs/TrafficSignArray")
(setf (get dcas_msgs::TrafficSignArray :definition-)
      "std_msgs/Header header
dcas_msgs/TrafficSign[] signs


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
MSG: dcas_msgs/TrafficSign
std_msgs/Header header
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



(provide :dcas_msgs/TrafficSignArray "06d9b79e9aad276f858f3ff080587c81")


