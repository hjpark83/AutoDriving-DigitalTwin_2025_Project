;; Auto-generated. Do not edit!


(when (boundp 'ros_tutorial::TutorialVehicleState)
  (if (not (find-package "ROS_TUTORIAL"))
    (make-package "ROS_TUTORIAL"))
  (shadow 'TutorialVehicleState (find-package "ROS_TUTORIAL")))
(unless (find-package "ROS_TUTORIAL::TUTORIALVEHICLESTATE")
  (make-package "ROS_TUTORIAL::TUTORIALVEHICLESTATE"))

(in-package "ROS")
;;//! \htmlinclude TutorialVehicleState.msg.html


(defclass ros_tutorial::TutorialVehicleState
  :super ros::object
  :slots (_x _y _yaw _steering_angle ))

(defmethod ros_tutorial::TutorialVehicleState
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:yaw __yaw) 0.0)
    ((:steering_angle __steering_angle) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _yaw (float __yaw))
   (setq _steering_angle (float __steering_angle))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:steering_angle
   (&optional __steering_angle)
   (if __steering_angle (setq _steering_angle __steering_angle)) _steering_angle)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _yaw
    8
    ;; float64 _steering_angle
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _steering_angle
       (sys::poke _steering_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _steering_angle
     (setq _steering_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get ros_tutorial::TutorialVehicleState :md5sum-) "4b6a4b07a703d3ad2519e2fcecc39a07")
(setf (get ros_tutorial::TutorialVehicleState :datatype-) "ros_tutorial/TutorialVehicleState")
(setf (get ros_tutorial::TutorialVehicleState :definition-)
      "float64 x
float64 y
float64 yaw
float64 steering_angle

")



(provide :ros_tutorial/TutorialVehicleState "4b6a4b07a703d3ad2519e2fcecc39a07")


