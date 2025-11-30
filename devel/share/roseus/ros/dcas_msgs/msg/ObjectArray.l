;; Auto-generated. Do not edit!


(when (boundp 'dcas_msgs::ObjectArray)
  (if (not (find-package "DCAS_MSGS"))
    (make-package "DCAS_MSGS"))
  (shadow 'ObjectArray (find-package "DCAS_MSGS")))
(unless (find-package "DCAS_MSGS::OBJECTARRAY")
  (make-package "DCAS_MSGS::OBJECTARRAY"))

(in-package "ROS")
;;//! \htmlinclude ObjectArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass dcas_msgs::ObjectArray
  :super ros::object
  :slots (_header _objects ))

(defmethod dcas_msgs::ObjectArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:objects __objects) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _objects __objects)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:objects
   (&rest __objects)
   (if (keywordp (car __objects))
       (send* _objects __objects)
     (progn
       (if __objects (setq _objects (car __objects)))
       _objects)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; dcas_msgs/Object[] _objects
    (apply #'+ (send-all _objects :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; dcas_msgs/Object[] _objects
     (write-long (length _objects) s)
     (dolist (elem _objects)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; dcas_msgs/Object[] _objects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _objects (let (r) (dotimes (i n) (push (instance dcas_msgs::Object :init) r)) r))
     (dolist (elem- _objects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get dcas_msgs::ObjectArray :md5sum-) "1a148595d8713c4b90d42f0a686db8d9")
(setf (get dcas_msgs::ObjectArray :datatype-) "dcas_msgs/ObjectArray")
(setf (get dcas_msgs::ObjectArray :definition-)
      "std_msgs/Header header
dcas_msgs/Object[] objects



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
MSG: dcas_msgs/Object
# Generic object description for environment/perception
std_msgs/Header header
uint32 id
uint8 type
geometry_msgs/Pose pose
geometry_msgs/Vector3 size  # For GEOM_BOX (fallback; width(x), depth(y), height(z))
geometry_msgs/Twist twist

# Geometry shape (to visualize true footprint, e.g., buildings)
uint8 GEOM_BOX=0
uint8 GEOM_POLYGON=1
uint8 geom_type
geometry_msgs/Point32[] footprint  # Map-frame polygon (z ignored); closed by convention (first==last not required)
float32 height  # For GEOM_POLYGON, extruded height (meters)

# Type labels (keep minimal and course-focused)
uint8 LABEL_UNKNOWN=0
uint8 LABEL_VEHICLE=1
uint8 LABEL_PEDESTRIAN=2
uint8 LABEL_CYCLIST=3
uint8 LABEL_TRAFFIC_LIGHT=10
uint8 LABEL_TRAFFIC_SIGN=11
uint8 LABEL_GUARDRAIL=12
uint8 LABEL_BUILDING=13



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

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3 linear
Vector3 angular

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



(provide :dcas_msgs/ObjectArray "1a148595d8713c4b90d42f0a686db8d9")


