
(cl:in-package :asdf)

(defsystem "phasespace_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Camera" :depends-on ("_package_Camera"))
    (:file "_package_Camera" :depends-on ("_package"))
    (:file "Cameras" :depends-on ("_package_Cameras"))
    (:file "_package_Cameras" :depends-on ("_package"))
    (:file "Marker" :depends-on ("_package_Marker"))
    (:file "_package_Marker" :depends-on ("_package"))
    (:file "Markers" :depends-on ("_package_Markers"))
    (:file "_package_Markers" :depends-on ("_package"))
    (:file "Rigid" :depends-on ("_package_Rigid"))
    (:file "_package_Rigid" :depends-on ("_package"))
    (:file "Rigids" :depends-on ("_package_Rigids"))
    (:file "_package_Rigids" :depends-on ("_package"))
  ))