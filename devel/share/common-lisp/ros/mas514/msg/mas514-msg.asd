
(cl:in-package :asdf)

(defsystem "mas514-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ServoSetpoints" :depends-on ("_package_ServoSetpoints"))
    (:file "_package_ServoSetpoints" :depends-on ("_package"))
    (:file "WebJoystick" :depends-on ("_package_WebJoystick"))
    (:file "_package_WebJoystick" :depends-on ("_package"))
  ))