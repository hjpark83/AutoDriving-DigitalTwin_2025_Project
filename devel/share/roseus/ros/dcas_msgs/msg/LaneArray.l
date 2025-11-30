;; Auto-generated. Do not edit!


(when (boundp 'dcas_msgs::LaneArray)
  (if (not (find-package "DCAS_MSGS"))
    (make-package "DCAS_MSGS"))
  (shadow 'LaneArray (find-package "DCAS_MSGS")))
(unless (find-package "DCAS_MSGS::LANEARRAY")
  (make-package "DCAS_MSGS::LANEARRAY"))

(in-package "ROS")
;;//! \htmlinclude LaneArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass dcas_msgs::LaneArray
  :super ros::object
  :slots (_header _lanes ))

(defmethod dcas_msgs::LaneArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lanes __lanes) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _lanes __lanes)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lanes
   (&rest __lanes)
   (if (keywordp (car __lanes))
       (send* _lanes __lanes)
     (progn
       (if __lanes (setq _lanes (car __lanes)))
       _lanes)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; dcas_msgs/Lane[] _lanes
    (apply #'+ (send-all _lanes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; dcas_msgs/Lane[] _lanes
     (write-long (length _lanes) s)
     (dolist (elem _lanes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; dcas_msgs/Lane[] _lanes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _lanes (let (r) (dotimes (i n) (push (instance dcas_msgs::Lane :init) r)) r))
     (dolist (elem- _lanes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get dcas_msgs::LaneArray :md5sum-) "d2a6ebf6f2f2918e36b22f22ed98e619")
(setf (get dcas_msgs::LaneArray :datatype-) "dcas_msgs/LaneArray")
(setf (get dcas_msgs::LaneArray :definition-)
      "std_msgs/Header header
dcas_msgs/Lane[] lanes



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
MSG: dcas_msgs/Lane
std_msgs/Header header
uint32 id
# For simplicity use 2D points in map frame
geometry_msgs/Point32[] lane_lines

# Lane type definitions
uint8 TYPE_UNKNOWN=0
uint8 TYPE_WHITE_SOLID=1
uint8 TYPE_WHITE_DASHED=2
uint8 TYPE_YELLOW_SOLID=3
uint8 TYPE_YELLOW_DASHED=4
uint8 TYPE_DOUBLE_YELLOW_SOLID=5
uint8 lane_type



================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z
")



(provide :dcas_msgs/LaneArray "d2a6ebf6f2f2918e36b22f22ed98e619")


