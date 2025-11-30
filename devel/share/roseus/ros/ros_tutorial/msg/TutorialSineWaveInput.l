;; Auto-generated. Do not edit!


(when (boundp 'ros_tutorial::TutorialSineWaveInput)
  (if (not (find-package "ROS_TUTORIAL"))
    (make-package "ROS_TUTORIAL"))
  (shadow 'TutorialSineWaveInput (find-package "ROS_TUTORIAL")))
(unless (find-package "ROS_TUTORIAL::TUTORIALSINEWAVEINPUT")
  (make-package "ROS_TUTORIAL::TUTORIALSINEWAVEINPUT"))

(in-package "ROS")
;;//! \htmlinclude TutorialSineWaveInput.msg.html


(defclass ros_tutorial::TutorialSineWaveInput
  :super ros::object
  :slots (_period _noise_standard_deviation _amplitude ))

(defmethod ros_tutorial::TutorialSineWaveInput
  (:init
   (&key
    ((:period __period) 0.0)
    ((:noise_standard_deviation __noise_standard_deviation) 0.0)
    ((:amplitude __amplitude) 0.0)
    )
   (send-super :init)
   (setq _period (float __period))
   (setq _noise_standard_deviation (float __noise_standard_deviation))
   (setq _amplitude (float __amplitude))
   self)
  (:period
   (&optional __period)
   (if __period (setq _period __period)) _period)
  (:noise_standard_deviation
   (&optional __noise_standard_deviation)
   (if __noise_standard_deviation (setq _noise_standard_deviation __noise_standard_deviation)) _noise_standard_deviation)
  (:amplitude
   (&optional __amplitude)
   (if __amplitude (setq _amplitude __amplitude)) _amplitude)
  (:serialization-length
   ()
   (+
    ;; float64 _period
    8
    ;; float64 _noise_standard_deviation
    8
    ;; float64 _amplitude
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _period
       (sys::poke _period (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _noise_standard_deviation
       (sys::poke _noise_standard_deviation (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _amplitude
       (sys::poke _amplitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _period
     (setq _period (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _noise_standard_deviation
     (setq _noise_standard_deviation (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _amplitude
     (setq _amplitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get ros_tutorial::TutorialSineWaveInput :md5sum-) "5cd899687a4aae1254b2449fcada7c3e")
(setf (get ros_tutorial::TutorialSineWaveInput :datatype-) "ros_tutorial/TutorialSineWaveInput")
(setf (get ros_tutorial::TutorialSineWaveInput :definition-)
      "float64 period
float64 noise_standard_deviation
float64 amplitude

")



(provide :ros_tutorial/TutorialSineWaveInput "5cd899687a4aae1254b2449fcada7c3e")


