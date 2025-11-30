; Auto-generated. Do not edit!


(cl:in-package ros_tutorial-msg)


;//! \htmlinclude TutorialSineWaveInput.msg.html

(cl:defclass <TutorialSineWaveInput> (roslisp-msg-protocol:ros-message)
  ((period
    :reader period
    :initarg :period
    :type cl:float
    :initform 0.0)
   (noise_standard_deviation
    :reader noise_standard_deviation
    :initarg :noise_standard_deviation
    :type cl:float
    :initform 0.0)
   (amplitude
    :reader amplitude
    :initarg :amplitude
    :type cl:float
    :initform 0.0))
)

(cl:defclass TutorialSineWaveInput (<TutorialSineWaveInput>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TutorialSineWaveInput>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TutorialSineWaveInput)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_tutorial-msg:<TutorialSineWaveInput> is deprecated: use ros_tutorial-msg:TutorialSineWaveInput instead.")))

(cl:ensure-generic-function 'period-val :lambda-list '(m))
(cl:defmethod period-val ((m <TutorialSineWaveInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_tutorial-msg:period-val is deprecated.  Use ros_tutorial-msg:period instead.")
  (period m))

(cl:ensure-generic-function 'noise_standard_deviation-val :lambda-list '(m))
(cl:defmethod noise_standard_deviation-val ((m <TutorialSineWaveInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_tutorial-msg:noise_standard_deviation-val is deprecated.  Use ros_tutorial-msg:noise_standard_deviation instead.")
  (noise_standard_deviation m))

(cl:ensure-generic-function 'amplitude-val :lambda-list '(m))
(cl:defmethod amplitude-val ((m <TutorialSineWaveInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_tutorial-msg:amplitude-val is deprecated.  Use ros_tutorial-msg:amplitude instead.")
  (amplitude m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TutorialSineWaveInput>) ostream)
  "Serializes a message object of type '<TutorialSineWaveInput>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'period))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'noise_standard_deviation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'amplitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TutorialSineWaveInput>) istream)
  "Deserializes a message object of type '<TutorialSineWaveInput>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'period) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'noise_standard_deviation) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'amplitude) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TutorialSineWaveInput>)))
  "Returns string type for a message object of type '<TutorialSineWaveInput>"
  "ros_tutorial/TutorialSineWaveInput")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TutorialSineWaveInput)))
  "Returns string type for a message object of type 'TutorialSineWaveInput"
  "ros_tutorial/TutorialSineWaveInput")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TutorialSineWaveInput>)))
  "Returns md5sum for a message object of type '<TutorialSineWaveInput>"
  "5cd899687a4aae1254b2449fcada7c3e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TutorialSineWaveInput)))
  "Returns md5sum for a message object of type 'TutorialSineWaveInput"
  "5cd899687a4aae1254b2449fcada7c3e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TutorialSineWaveInput>)))
  "Returns full string definition for message of type '<TutorialSineWaveInput>"
  (cl:format cl:nil "float64 period~%float64 noise_standard_deviation~%float64 amplitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TutorialSineWaveInput)))
  "Returns full string definition for message of type 'TutorialSineWaveInput"
  (cl:format cl:nil "float64 period~%float64 noise_standard_deviation~%float64 amplitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TutorialSineWaveInput>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TutorialSineWaveInput>))
  "Converts a ROS message object to a list"
  (cl:list 'TutorialSineWaveInput
    (cl:cons ':period (period msg))
    (cl:cons ':noise_standard_deviation (noise_standard_deviation msg))
    (cl:cons ':amplitude (amplitude msg))
))
