
(cl:in-package :asdf)

(defsystem "ros_tutorial-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "TutorialSineWaveInput" :depends-on ("_package_TutorialSineWaveInput"))
    (:file "_package_TutorialSineWaveInput" :depends-on ("_package"))
    (:file "TutorialVehicleState" :depends-on ("_package_TutorialVehicleState"))
    (:file "_package_TutorialVehicleState" :depends-on ("_package"))
  ))