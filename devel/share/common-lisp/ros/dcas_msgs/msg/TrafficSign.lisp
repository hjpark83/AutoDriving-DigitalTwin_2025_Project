; Auto-generated. Do not edit!


(cl:in-package dcas_msgs-msg)


;//! \htmlinclude TrafficSign.msg.html

(cl:defclass <TrafficSign> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (sign_class
    :reader sign_class
    :initarg :sign_class
    :type cl:fixnum
    :initform 0))
)

(cl:defclass TrafficSign (<TrafficSign>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrafficSign>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrafficSign)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dcas_msgs-msg:<TrafficSign> is deprecated: use dcas_msgs-msg:TrafficSign instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrafficSign>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dcas_msgs-msg:header-val is deprecated.  Use dcas_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <TrafficSign>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dcas_msgs-msg:id-val is deprecated.  Use dcas_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <TrafficSign>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dcas_msgs-msg:pose-val is deprecated.  Use dcas_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'sign_class-val :lambda-list '(m))
(cl:defmethod sign_class-val ((m <TrafficSign>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dcas_msgs-msg:sign_class-val is deprecated.  Use dcas_msgs-msg:sign_class instead.")
  (sign_class m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<TrafficSign>)))
    "Constants for message type '<TrafficSign>"
  '((:CLASS_UNKNOWN . 0)
    (:CLASS_STOP . 1)
    (:CLASS_YIELD . 2)
    (:CLASS_SPEED_LIMIT_30 . 3)
    (:CLASS_SPEED_LIMIT_50 . 4)
    (:CLASS_SPEED_LIMIT_60 . 5)
    (:CLASS_SPEED_LIMIT_80 . 6)
    (:CLASS_NO_ENTRY . 7)
    (:CLASS_NO_PARKING . 8)
    (:CLASS_PEDESTRIAN_CROSSING . 9)
    (:CLASS_SCHOOL_ZONE . 10)
    (:CLASS_CONSTRUCTION . 11)
    (:CLASS_TURN_LEFT . 12)
    (:CLASS_TURN_RIGHT . 13)
    (:CLASS_GO_STRAIGHT . 14))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'TrafficSign)))
    "Constants for message type 'TrafficSign"
  '((:CLASS_UNKNOWN . 0)
    (:CLASS_STOP . 1)
    (:CLASS_YIELD . 2)
    (:CLASS_SPEED_LIMIT_30 . 3)
    (:CLASS_SPEED_LIMIT_50 . 4)
    (:CLASS_SPEED_LIMIT_60 . 5)
    (:CLASS_SPEED_LIMIT_80 . 6)
    (:CLASS_NO_ENTRY . 7)
    (:CLASS_NO_PARKING . 8)
    (:CLASS_PEDESTRIAN_CROSSING . 9)
    (:CLASS_SCHOOL_ZONE . 10)
    (:CLASS_CONSTRUCTION . 11)
    (:CLASS_TURN_LEFT . 12)
    (:CLASS_TURN_RIGHT . 13)
    (:CLASS_GO_STRAIGHT . 14))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrafficSign>) ostream)
  "Serializes a message object of type '<TrafficSign>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sign_class)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrafficSign>) istream)
  "Deserializes a message object of type '<TrafficSign>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sign_class)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrafficSign>)))
  "Returns string type for a message object of type '<TrafficSign>"
  "dcas_msgs/TrafficSign")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrafficSign)))
  "Returns string type for a message object of type 'TrafficSign"
  "dcas_msgs/TrafficSign")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrafficSign>)))
  "Returns md5sum for a message object of type '<TrafficSign>"
  "c95b6f353f8d2898545192a2a0de59a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrafficSign)))
  "Returns md5sum for a message object of type 'TrafficSign"
  "c95b6f353f8d2898545192a2a0de59a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrafficSign>)))
  "Returns full string definition for message of type '<TrafficSign>"
  (cl:format cl:nil "std_msgs/Header header~%uint32 id~%geometry_msgs/Pose pose~%uint8 sign_class~%~%# Traffic sign classes~%uint8 CLASS_UNKNOWN=0~%uint8 CLASS_STOP=1~%uint8 CLASS_YIELD=2~%uint8 CLASS_SPEED_LIMIT_30=3~%uint8 CLASS_SPEED_LIMIT_50=4~%uint8 CLASS_SPEED_LIMIT_60=5~%uint8 CLASS_SPEED_LIMIT_80=6~%uint8 CLASS_NO_ENTRY=7~%uint8 CLASS_NO_PARKING=8~%uint8 CLASS_PEDESTRIAN_CROSSING=9~%uint8 CLASS_SCHOOL_ZONE=10~%uint8 CLASS_CONSTRUCTION=11~%uint8 CLASS_TURN_LEFT=12~%uint8 CLASS_TURN_RIGHT=13~%uint8 CLASS_GO_STRAIGHT=14~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrafficSign)))
  "Returns full string definition for message of type 'TrafficSign"
  (cl:format cl:nil "std_msgs/Header header~%uint32 id~%geometry_msgs/Pose pose~%uint8 sign_class~%~%# Traffic sign classes~%uint8 CLASS_UNKNOWN=0~%uint8 CLASS_STOP=1~%uint8 CLASS_YIELD=2~%uint8 CLASS_SPEED_LIMIT_30=3~%uint8 CLASS_SPEED_LIMIT_50=4~%uint8 CLASS_SPEED_LIMIT_60=5~%uint8 CLASS_SPEED_LIMIT_80=6~%uint8 CLASS_NO_ENTRY=7~%uint8 CLASS_NO_PARKING=8~%uint8 CLASS_PEDESTRIAN_CROSSING=9~%uint8 CLASS_SCHOOL_ZONE=10~%uint8 CLASS_CONSTRUCTION=11~%uint8 CLASS_TURN_LEFT=12~%uint8 CLASS_TURN_RIGHT=13~%uint8 CLASS_GO_STRAIGHT=14~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrafficSign>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrafficSign>))
  "Converts a ROS message object to a list"
  (cl:list 'TrafficSign
    (cl:cons ':header (header msg))
    (cl:cons ':id (id msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':sign_class (sign_class msg))
))
