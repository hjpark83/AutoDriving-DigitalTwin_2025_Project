;; Auto-generated. Do not edit!


(when (boundp 'dcas_msgs::Object)
  (if (not (find-package "DCAS_MSGS"))
    (make-package "DCAS_MSGS"))
  (shadow 'Object (find-package "DCAS_MSGS")))
(unless (find-package "DCAS_MSGS::OBJECT")
  (make-package "DCAS_MSGS::OBJECT"))

(in-package "ROS")
;;//! \htmlinclude Object.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*GEOM_BOX*" (find-package "DCAS_MSGS::OBJECT"))
(shadow '*GEOM_BOX* (find-package "DCAS_MSGS::OBJECT"))
(defconstant dcas_msgs::Object::*GEOM_BOX* 0)
(intern "*GEOM_POLYGON*" (find-package "DCAS_MSGS::OBJECT"))
(shadow '*GEOM_POLYGON* (find-package "DCAS_MSGS::OBJECT"))
(defconstant dcas_msgs::Object::*GEOM_POLYGON* 1)
(intern "*LABEL_UNKNOWN*" (find-package "DCAS_MSGS::OBJECT"))
(shadow '*LABEL_UNKNOWN* (find-package "DCAS_MSGS::OBJECT"))
(defconstant dcas_msgs::Object::*LABEL_UNKNOWN* 0)
(intern "*LABEL_VEHICLE*" (find-package "DCAS_MSGS::OBJECT"))
(shadow '*LABEL_VEHICLE* (find-package "DCAS_MSGS::OBJECT"))
(defconstant dcas_msgs::Object::*LABEL_VEHICLE* 1)
(intern "*LABEL_PEDESTRIAN*" (find-package "DCAS_MSGS::OBJECT"))
(shadow '*LABEL_PEDESTRIAN* (find-package "DCAS_MSGS::OBJECT"))
(defconstant dcas_msgs::Object::*LABEL_PEDESTRIAN* 2)
(intern "*LABEL_CYCLIST*" (find-package "DCAS_MSGS::OBJECT"))
(shadow '*LABEL_CYCLIST* (find-package "DCAS_MSGS::OBJECT"))
(defconstant dcas_msgs::Object::*LABEL_CYCLIST* 3)
(intern "*LABEL_TRAFFIC_LIGHT*" (find-package "DCAS_MSGS::OBJECT"))
(shadow '*LABEL_TRAFFIC_LIGHT* (find-package "DCAS_MSGS::OBJECT"))
(defconstant dcas_msgs::Object::*LABEL_TRAFFIC_LIGHT* 10)
(intern "*LABEL_TRAFFIC_SIGN*" (find-package "DCAS_MSGS::OBJECT"))
(shadow '*LABEL_TRAFFIC_SIGN* (find-package "DCAS_MSGS::OBJECT"))
(defconstant dcas_msgs::Object::*LABEL_TRAFFIC_SIGN* 11)
(intern "*LABEL_GUARDRAIL*" (find-package "DCAS_MSGS::OBJECT"))
(shadow '*LABEL_GUARDRAIL* (find-package "DCAS_MSGS::OBJECT"))
(defconstant dcas_msgs::Object::*LABEL_GUARDRAIL* 12)
(intern "*LABEL_BUILDING*" (find-package "DCAS_MSGS::OBJECT"))
(shadow '*LABEL_BUILDING* (find-package "DCAS_MSGS::OBJECT"))
(defconstant dcas_msgs::Object::*LABEL_BUILDING* 13)

(defun dcas_msgs::Object-to-symbol (const)
  (cond
        ((= const 0) 'dcas_msgs::Object::*GEOM_BOX*)
        ((= const 1) 'dcas_msgs::Object::*GEOM_POLYGON*)
        ((= const 0) 'dcas_msgs::Object::*LABEL_UNKNOWN*)
        ((= const 1) 'dcas_msgs::Object::*LABEL_VEHICLE*)
        ((= const 2) 'dcas_msgs::Object::*LABEL_PEDESTRIAN*)
        ((= const 3) 'dcas_msgs::Object::*LABEL_CYCLIST*)
        ((= const 10) 'dcas_msgs::Object::*LABEL_TRAFFIC_LIGHT*)
        ((= const 11) 'dcas_msgs::Object::*LABEL_TRAFFIC_SIGN*)
        ((= const 12) 'dcas_msgs::Object::*LABEL_GUARDRAIL*)
        ((= const 13) 'dcas_msgs::Object::*LABEL_BUILDING*)
        (t nil)))

(defclass dcas_msgs::Object
  :super ros::object
  :slots (_header _id _type _pose _size _twist _geom_type _footprint _height ))

(defmethod dcas_msgs::Object
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id __id) 0)
    ((:type __type) 0)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:size __size) (instance geometry_msgs::Vector3 :init))
    ((:twist __twist) (instance geometry_msgs::Twist :init))
    ((:geom_type __geom_type) 0)
    ((:footprint __footprint) ())
    ((:height __height) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _id (round __id))
   (setq _type (round __type))
   (setq _pose __pose)
   (setq _size __size)
   (setq _twist __twist)
   (setq _geom_type (round __geom_type))
   (setq _footprint __footprint)
   (setq _height (float __height))
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
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:size
   (&rest __size)
   (if (keywordp (car __size))
       (send* _size __size)
     (progn
       (if __size (setq _size (car __size)))
       _size)))
  (:twist
   (&rest __twist)
   (if (keywordp (car __twist))
       (send* _twist __twist)
     (progn
       (if __twist (setq _twist (car __twist)))
       _twist)))
  (:geom_type
   (&optional __geom_type)
   (if __geom_type (setq _geom_type __geom_type)) _geom_type)
  (:footprint
   (&rest __footprint)
   (if (keywordp (car __footprint))
       (send* _footprint __footprint)
     (progn
       (if __footprint (setq _footprint (car __footprint)))
       _footprint)))
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _id
    4
    ;; uint8 _type
    1
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Vector3 _size
    (send _size :serialization-length)
    ;; geometry_msgs/Twist _twist
    (send _twist :serialization-length)
    ;; uint8 _geom_type
    1
    ;; geometry_msgs/Point32[] _footprint
    (apply #'+ (send-all _footprint :serialization-length)) 4
    ;; float32 _height
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _id
       (write-long _id s)
     ;; uint8 _type
       (write-byte _type s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Vector3 _size
       (send _size :serialize s)
     ;; geometry_msgs/Twist _twist
       (send _twist :serialize s)
     ;; uint8 _geom_type
       (write-byte _geom_type s)
     ;; geometry_msgs/Point32[] _footprint
     (write-long (length _footprint) s)
     (dolist (elem _footprint)
       (send elem :serialize s)
       )
     ;; float32 _height
       (sys::poke _height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Vector3 _size
     (send _size :deserialize buf ptr-) (incf ptr- (send _size :serialization-length))
   ;; geometry_msgs/Twist _twist
     (send _twist :deserialize buf ptr-) (incf ptr- (send _twist :serialization-length))
   ;; uint8 _geom_type
     (setq _geom_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Point32[] _footprint
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _footprint (let (r) (dotimes (i n) (push (instance geometry_msgs::Point32 :init) r)) r))
     (dolist (elem- _footprint)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32 _height
     (setq _height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dcas_msgs::Object :md5sum-) "09a2b3417990e01d5598408cceb6c012")
(setf (get dcas_msgs::Object :datatype-) "dcas_msgs/Object")
(setf (get dcas_msgs::Object :definition-)
      "# Generic object description for environment/perception
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



(provide :dcas_msgs/Object "09a2b3417990e01d5598408cceb6c012")


