; Auto-generated. Do not edit!


(cl:in-package mas514-msg)


;//! \htmlinclude ServoSetpoints.msg.html

(cl:defclass <ServoSetpoints> (roslisp-msg-protocol:ros-message)
  ((rightWheel
    :reader rightWheel
    :initarg :rightWheel
    :type cl:integer
    :initform 0)
   (leftWheel
    :reader leftWheel
    :initarg :leftWheel
    :type cl:integer
    :initform 0)
   (servo1
    :reader servo1
    :initarg :servo1
    :type cl:integer
    :initform 0)
   (servo2
    :reader servo2
    :initarg :servo2
    :type cl:integer
    :initform 0)
   (servo3
    :reader servo3
    :initarg :servo3
    :type cl:integer
    :initform 0))
)

(cl:defclass ServoSetpoints (<ServoSetpoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ServoSetpoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ServoSetpoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mas514-msg:<ServoSetpoints> is deprecated: use mas514-msg:ServoSetpoints instead.")))

(cl:ensure-generic-function 'rightWheel-val :lambda-list '(m))
(cl:defmethod rightWheel-val ((m <ServoSetpoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mas514-msg:rightWheel-val is deprecated.  Use mas514-msg:rightWheel instead.")
  (rightWheel m))

(cl:ensure-generic-function 'leftWheel-val :lambda-list '(m))
(cl:defmethod leftWheel-val ((m <ServoSetpoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mas514-msg:leftWheel-val is deprecated.  Use mas514-msg:leftWheel instead.")
  (leftWheel m))

(cl:ensure-generic-function 'servo1-val :lambda-list '(m))
(cl:defmethod servo1-val ((m <ServoSetpoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mas514-msg:servo1-val is deprecated.  Use mas514-msg:servo1 instead.")
  (servo1 m))

(cl:ensure-generic-function 'servo2-val :lambda-list '(m))
(cl:defmethod servo2-val ((m <ServoSetpoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mas514-msg:servo2-val is deprecated.  Use mas514-msg:servo2 instead.")
  (servo2 m))

(cl:ensure-generic-function 'servo3-val :lambda-list '(m))
(cl:defmethod servo3-val ((m <ServoSetpoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mas514-msg:servo3-val is deprecated.  Use mas514-msg:servo3 instead.")
  (servo3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ServoSetpoints>) ostream)
  "Serializes a message object of type '<ServoSetpoints>"
  (cl:let* ((signed (cl:slot-value msg 'rightWheel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'leftWheel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'servo1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'servo2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'servo3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ServoSetpoints>) istream)
  "Deserializes a message object of type '<ServoSetpoints>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rightWheel) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'leftWheel) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'servo1) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'servo2) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'servo3) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ServoSetpoints>)))
  "Returns string type for a message object of type '<ServoSetpoints>"
  "mas514/ServoSetpoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ServoSetpoints)))
  "Returns string type for a message object of type 'ServoSetpoints"
  "mas514/ServoSetpoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ServoSetpoints>)))
  "Returns md5sum for a message object of type '<ServoSetpoints>"
  "790ff8777d41a4ebf31d8435d32b565b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ServoSetpoints)))
  "Returns md5sum for a message object of type 'ServoSetpoints"
  "790ff8777d41a4ebf31d8435d32b565b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ServoSetpoints>)))
  "Returns full string definition for message of type '<ServoSetpoints>"
  (cl:format cl:nil "int64 rightWheel~%int64 leftWheel~%int64 servo1~%int64 servo2~%int64 servo3~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ServoSetpoints)))
  "Returns full string definition for message of type 'ServoSetpoints"
  (cl:format cl:nil "int64 rightWheel~%int64 leftWheel~%int64 servo1~%int64 servo2~%int64 servo3~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ServoSetpoints>))
  (cl:+ 0
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ServoSetpoints>))
  "Converts a ROS message object to a list"
  (cl:list 'ServoSetpoints
    (cl:cons ':rightWheel (rightWheel msg))
    (cl:cons ':leftWheel (leftWheel msg))
    (cl:cons ':servo1 (servo1 msg))
    (cl:cons ':servo2 (servo2 msg))
    (cl:cons ':servo3 (servo3 msg))
))
