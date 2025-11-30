; Auto-generated. Do not edit!


(cl:in-package dcas_msgs-msg)


;//! \htmlinclude RadarDetectionArray.msg.html

(cl:defclass <RadarDetectionArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (detections
    :reader detections
    :initarg :detections
    :type (cl:vector dcas_msgs-msg:RadarDetection)
   :initform (cl:make-array 0 :element-type 'dcas_msgs-msg:RadarDetection :initial-element (cl:make-instance 'dcas_msgs-msg:RadarDetection))))
)

(cl:defclass RadarDetectionArray (<RadarDetectionArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RadarDetectionArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RadarDetectionArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dcas_msgs-msg:<RadarDetectionArray> is deprecated: use dcas_msgs-msg:RadarDetectionArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RadarDetectionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dcas_msgs-msg:header-val is deprecated.  Use dcas_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <RadarDetectionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dcas_msgs-msg:detections-val is deprecated.  Use dcas_msgs-msg:detections instead.")
  (detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RadarDetectionArray>) ostream)
  "Serializes a message object of type '<RadarDetectionArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RadarDetectionArray>) istream)
  "Deserializes a message object of type '<RadarDetectionArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'dcas_msgs-msg:RadarDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RadarDetectionArray>)))
  "Returns string type for a message object of type '<RadarDetectionArray>"
  "dcas_msgs/RadarDetectionArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RadarDetectionArray)))
  "Returns string type for a message object of type 'RadarDetectionArray"
  "dcas_msgs/RadarDetectionArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RadarDetectionArray>)))
  "Returns md5sum for a message object of type '<RadarDetectionArray>"
  "fda954c0e5481cab05327c93c07e585f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RadarDetectionArray)))
  "Returns md5sum for a message object of type 'RadarDetectionArray"
  "fda954c0e5481cab05327c93c07e585f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RadarDetectionArray>)))
  "Returns full string definition for message of type '<RadarDetectionArray>"
  (cl:format cl:nil "std_msgs/Header header~%dcas_msgs/RadarDetection[] detections~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: dcas_msgs/RadarDetection~%std_msgs/Header header~%geometry_msgs/Point position~%geometry_msgs/Vector3 velocity~%float32 rcs~%float32 range~%float32 doppler~%~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RadarDetectionArray)))
  "Returns full string definition for message of type 'RadarDetectionArray"
  (cl:format cl:nil "std_msgs/Header header~%dcas_msgs/RadarDetection[] detections~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: dcas_msgs/RadarDetection~%std_msgs/Header header~%geometry_msgs/Point position~%geometry_msgs/Vector3 velocity~%float32 rcs~%float32 range~%float32 doppler~%~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RadarDetectionArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RadarDetectionArray>))
  "Converts a ROS message object to a list"
  (cl:list 'RadarDetectionArray
    (cl:cons ':header (header msg))
    (cl:cons ':detections (detections msg))
))
