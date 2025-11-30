;; Auto-generated. Do not edit!


(when (boundp 'dcas_msgs::VehicleState)
  (if (not (find-package "DCAS_MSGS"))
    (make-package "DCAS_MSGS"))
  (shadow 'VehicleState (find-package "DCAS_MSGS")))
(unless (find-package "DCAS_MSGS::VEHICLESTATE")
  (make-package "DCAS_MSGS::VEHICLESTATE"))

(in-package "ROS")
;;//! \htmlinclude VehicleState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass dcas_msgs::VehicleState
  :super ros::object
  :slots (_header _pose _twist _acceleration _steering_wheel_angle_deg _wheel_speeds_mps _accelerator_pedal _brake_pedal ))

(defmethod dcas_msgs::VehicleState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:twist __twist) (instance geometry_msgs::Twist :init))
    ((:acceleration __acceleration) (instance geometry_msgs::Vector3 :init))
    ((:steering_wheel_angle_deg __steering_wheel_angle_deg) 0.0)
    ((:wheel_speeds_mps __wheel_speeds_mps) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:accelerator_pedal __accelerator_pedal) 0.0)
    ((:brake_pedal __brake_pedal) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _pose __pose)
   (setq _twist __twist)
   (setq _acceleration __acceleration)
   (setq _steering_wheel_angle_deg (float __steering_wheel_angle_deg))
   (setq _wheel_speeds_mps __wheel_speeds_mps)
   (setq _accelerator_pedal (float __accelerator_pedal))
   (setq _brake_pedal (float __brake_pedal))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:twist
   (&rest __twist)
   (if (keywordp (car __twist))
       (send* _twist __twist)
     (progn
       (if __twist (setq _twist (car __twist)))
       _twist)))
  (:acceleration
   (&rest __acceleration)
   (if (keywordp (car __acceleration))
       (send* _acceleration __acceleration)
     (progn
       (if __acceleration (setq _acceleration (car __acceleration)))
       _acceleration)))
  (:steering_wheel_angle_deg
   (&optional __steering_wheel_angle_deg)
   (if __steering_wheel_angle_deg (setq _steering_wheel_angle_deg __steering_wheel_angle_deg)) _steering_wheel_angle_deg)
  (:wheel_speeds_mps
   (&optional __wheel_speeds_mps)
   (if __wheel_speeds_mps (setq _wheel_speeds_mps __wheel_speeds_mps)) _wheel_speeds_mps)
  (:accelerator_pedal
   (&optional __accelerator_pedal)
   (if __accelerator_pedal (setq _accelerator_pedal __accelerator_pedal)) _accelerator_pedal)
  (:brake_pedal
   (&optional __brake_pedal)
   (if __brake_pedal (setq _brake_pedal __brake_pedal)) _brake_pedal)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Twist _twist
    (send _twist :serialization-length)
    ;; geometry_msgs/Vector3 _acceleration
    (send _acceleration :serialization-length)
    ;; float32 _steering_wheel_angle_deg
    4
    ;; float32[] _wheel_speeds_mps
    (* 4    (length _wheel_speeds_mps)) 4
    ;; float32 _accelerator_pedal
    4
    ;; float32 _brake_pedal
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Twist _twist
       (send _twist :serialize s)
     ;; geometry_msgs/Vector3 _acceleration
       (send _acceleration :serialize s)
     ;; float32 _steering_wheel_angle_deg
       (sys::poke _steering_wheel_angle_deg (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[] _wheel_speeds_mps
     (write-long (length _wheel_speeds_mps) s)
     (dotimes (i (length _wheel_speeds_mps))
       (sys::poke (elt _wheel_speeds_mps i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _accelerator_pedal
       (sys::poke _accelerator_pedal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _brake_pedal
       (sys::poke _brake_pedal (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Twist _twist
     (send _twist :deserialize buf ptr-) (incf ptr- (send _twist :serialization-length))
   ;; geometry_msgs/Vector3 _acceleration
     (send _acceleration :deserialize buf ptr-) (incf ptr- (send _acceleration :serialization-length))
   ;; float32 _steering_wheel_angle_deg
     (setq _steering_wheel_angle_deg (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[] _wheel_speeds_mps
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _wheel_speeds_mps (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _wheel_speeds_mps i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _accelerator_pedal
     (setq _accelerator_pedal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _brake_pedal
     (setq _brake_pedal (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get dcas_msgs::VehicleState :md5sum-) "18b3a6879ecb65c8c32b4cb5c098ed3c")
(setf (get dcas_msgs::VehicleState :datatype-) "dcas_msgs/VehicleState")
(setf (get dcas_msgs::VehicleState :definition-)
      "std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Vector3 acceleration
float32 steering_wheel_angle_deg
float32[] wheel_speeds_mps
float32 accelerator_pedal
float32 brake_pedal



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
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3 linear
Vector3 angular

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
")



(provide :dcas_msgs/VehicleState "18b3a6879ecb65c8c32b4cb5c098ed3c")


