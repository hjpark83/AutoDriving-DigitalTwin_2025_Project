;; Auto-generated. Do not edit!


(when (boundp 'dcas_msgs::Lane)
  (if (not (find-package "DCAS_MSGS"))
    (make-package "DCAS_MSGS"))
  (shadow 'Lane (find-package "DCAS_MSGS")))
(unless (find-package "DCAS_MSGS::LANE")
  (make-package "DCAS_MSGS::LANE"))

(in-package "ROS")
;;//! \htmlinclude Lane.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*TYPE_UNKNOWN*" (find-package "DCAS_MSGS::LANE"))
(shadow '*TYPE_UNKNOWN* (find-package "DCAS_MSGS::LANE"))
(defconstant dcas_msgs::Lane::*TYPE_UNKNOWN* 0)
(intern "*TYPE_WHITE_SOLID*" (find-package "DCAS_MSGS::LANE"))
(shadow '*TYPE_WHITE_SOLID* (find-package "DCAS_MSGS::LANE"))
(defconstant dcas_msgs::Lane::*TYPE_WHITE_SOLID* 1)
(intern "*TYPE_WHITE_DASHED*" (find-package "DCAS_MSGS::LANE"))
(shadow '*TYPE_WHITE_DASHED* (find-package "DCAS_MSGS::LANE"))
(defconstant dcas_msgs::Lane::*TYPE_WHITE_DASHED* 2)
(intern "*TYPE_YELLOW_SOLID*" (find-package "DCAS_MSGS::LANE"))
(shadow '*TYPE_YELLOW_SOLID* (find-package "DCAS_MSGS::LANE"))
(defconstant dcas_msgs::Lane::*TYPE_YELLOW_SOLID* 3)
(intern "*TYPE_YELLOW_DASHED*" (find-package "DCAS_MSGS::LANE"))
(shadow '*TYPE_YELLOW_DASHED* (find-package "DCAS_MSGS::LANE"))
(defconstant dcas_msgs::Lane::*TYPE_YELLOW_DASHED* 4)
(intern "*TYPE_DOUBLE_YELLOW_SOLID*" (find-package "DCAS_MSGS::LANE"))
(shadow '*TYPE_DOUBLE_YELLOW_SOLID* (find-package "DCAS_MSGS::LANE"))
(defconstant dcas_msgs::Lane::*TYPE_DOUBLE_YELLOW_SOLID* 5)

(defun dcas_msgs::Lane-to-symbol (const)
  (cond
        ((= const 0) 'dcas_msgs::Lane::*TYPE_UNKNOWN*)
        ((= const 1) 'dcas_msgs::Lane::*TYPE_WHITE_SOLID*)
        ((= const 2) 'dcas_msgs::Lane::*TYPE_WHITE_DASHED*)
        ((= const 3) 'dcas_msgs::Lane::*TYPE_YELLOW_SOLID*)
        ((= const 4) 'dcas_msgs::Lane::*TYPE_YELLOW_DASHED*)
        ((= const 5) 'dcas_msgs::Lane::*TYPE_DOUBLE_YELLOW_SOLID*)
        (t nil)))

(defclass dcas_msgs::Lane
  :super ros::object
  :slots (_header _id _lane_lines _lane_type ))

(defmethod dcas_msgs::Lane
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id __id) 0)
    ((:lane_lines __lane_lines) ())
    ((:lane_type __lane_type) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _id (round __id))
   (setq _lane_lines __lane_lines)
   (setq _lane_type (round __lane_type))
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
  (:lane_lines
   (&rest __lane_lines)
   (if (keywordp (car __lane_lines))
       (send* _lane_lines __lane_lines)
     (progn
       (if __lane_lines (setq _lane_lines (car __lane_lines)))
       _lane_lines)))
  (:lane_type
   (&optional __lane_type)
   (if __lane_type (setq _lane_type __lane_type)) _lane_type)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _id
    4
    ;; geometry_msgs/Point32[] _lane_lines
    (apply #'+ (send-all _lane_lines :serialization-length)) 4
    ;; uint8 _lane_type
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
     ;; geometry_msgs/Point32[] _lane_lines
     (write-long (length _lane_lines) s)
     (dolist (elem _lane_lines)
       (send elem :serialize s)
       )
     ;; uint8 _lane_type
       (write-byte _lane_type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Point32[] _lane_lines
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _lane_lines (let (r) (dotimes (i n) (push (instance geometry_msgs::Point32 :init) r)) r))
     (dolist (elem- _lane_lines)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint8 _lane_type
     (setq _lane_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get dcas_msgs::Lane :md5sum-) "b7404a3f5e4db4c5f96339c55f4901f6")
(setf (get dcas_msgs::Lane :datatype-) "dcas_msgs/Lane")
(setf (get dcas_msgs::Lane :definition-)
      "std_msgs/Header header
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



(provide :dcas_msgs/Lane "b7404a3f5e4db4c5f96339c55f4901f6")


