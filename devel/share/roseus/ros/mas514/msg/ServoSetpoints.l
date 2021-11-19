;; Auto-generated. Do not edit!


(when (boundp 'mas514::ServoSetpoints)
  (if (not (find-package "MAS514"))
    (make-package "MAS514"))
  (shadow 'ServoSetpoints (find-package "MAS514")))
(unless (find-package "MAS514::SERVOSETPOINTS")
  (make-package "MAS514::SERVOSETPOINTS"))

(in-package "ROS")
;;//! \htmlinclude ServoSetpoints.msg.html


(defclass mas514::ServoSetpoints
  :super ros::object
  :slots (_rightWheel _leftWheel _servo1 _servo2 _servo3 ))

(defmethod mas514::ServoSetpoints
  (:init
   (&key
    ((:rightWheel __rightWheel) 0)
    ((:leftWheel __leftWheel) 0)
    ((:servo1 __servo1) 0)
    ((:servo2 __servo2) 0)
    ((:servo3 __servo3) 0)
    )
   (send-super :init)
   (setq _rightWheel (round __rightWheel))
   (setq _leftWheel (round __leftWheel))
   (setq _servo1 (round __servo1))
   (setq _servo2 (round __servo2))
   (setq _servo3 (round __servo3))
   self)
  (:rightWheel
   (&optional __rightWheel)
   (if __rightWheel (setq _rightWheel __rightWheel)) _rightWheel)
  (:leftWheel
   (&optional __leftWheel)
   (if __leftWheel (setq _leftWheel __leftWheel)) _leftWheel)
  (:servo1
   (&optional __servo1)
   (if __servo1 (setq _servo1 __servo1)) _servo1)
  (:servo2
   (&optional __servo2)
   (if __servo2 (setq _servo2 __servo2)) _servo2)
  (:servo3
   (&optional __servo3)
   (if __servo3 (setq _servo3 __servo3)) _servo3)
  (:serialization-length
   ()
   (+
    ;; int64 _rightWheel
    8
    ;; int64 _leftWheel
    8
    ;; int64 _servo1
    8
    ;; int64 _servo2
    8
    ;; int64 _servo3
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _rightWheel
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _rightWheel (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _rightWheel) (= (length (_rightWheel . bv)) 2)) ;; bignum
              (write-long (ash (elt (_rightWheel . bv) 0) 0) s)
              (write-long (ash (elt (_rightWheel . bv) 1) -1) s))
             ((and (class _rightWheel) (= (length (_rightWheel . bv)) 1)) ;; big1
              (write-long (elt (_rightWheel . bv) 0) s)
              (write-long (if (>= _rightWheel 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _rightWheel s)(write-long (if (>= _rightWheel 0) 0 #xffffffff) s)))
     ;; int64 _leftWheel
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _leftWheel (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _leftWheel) (= (length (_leftWheel . bv)) 2)) ;; bignum
              (write-long (ash (elt (_leftWheel . bv) 0) 0) s)
              (write-long (ash (elt (_leftWheel . bv) 1) -1) s))
             ((and (class _leftWheel) (= (length (_leftWheel . bv)) 1)) ;; big1
              (write-long (elt (_leftWheel . bv) 0) s)
              (write-long (if (>= _leftWheel 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _leftWheel s)(write-long (if (>= _leftWheel 0) 0 #xffffffff) s)))
     ;; int64 _servo1
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _servo1 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _servo1) (= (length (_servo1 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_servo1 . bv) 0) 0) s)
              (write-long (ash (elt (_servo1 . bv) 1) -1) s))
             ((and (class _servo1) (= (length (_servo1 . bv)) 1)) ;; big1
              (write-long (elt (_servo1 . bv) 0) s)
              (write-long (if (>= _servo1 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _servo1 s)(write-long (if (>= _servo1 0) 0 #xffffffff) s)))
     ;; int64 _servo2
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _servo2 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _servo2) (= (length (_servo2 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_servo2 . bv) 0) 0) s)
              (write-long (ash (elt (_servo2 . bv) 1) -1) s))
             ((and (class _servo2) (= (length (_servo2 . bv)) 1)) ;; big1
              (write-long (elt (_servo2 . bv) 0) s)
              (write-long (if (>= _servo2 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _servo2 s)(write-long (if (>= _servo2 0) 0 #xffffffff) s)))
     ;; int64 _servo3
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _servo3 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _servo3) (= (length (_servo3 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_servo3 . bv) 0) 0) s)
              (write-long (ash (elt (_servo3 . bv) 1) -1) s))
             ((and (class _servo3) (= (length (_servo3 . bv)) 1)) ;; big1
              (write-long (elt (_servo3 . bv) 0) s)
              (write-long (if (>= _servo3 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _servo3 s)(write-long (if (>= _servo3 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _rightWheel
#+(or :alpha :irix6 :x86_64)
      (setf _rightWheel (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _rightWheel (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _leftWheel
#+(or :alpha :irix6 :x86_64)
      (setf _leftWheel (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _leftWheel (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _servo1
#+(or :alpha :irix6 :x86_64)
      (setf _servo1 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _servo1 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _servo2
#+(or :alpha :irix6 :x86_64)
      (setf _servo2 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _servo2 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _servo3
#+(or :alpha :irix6 :x86_64)
      (setf _servo3 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _servo3 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get mas514::ServoSetpoints :md5sum-) "790ff8777d41a4ebf31d8435d32b565b")
(setf (get mas514::ServoSetpoints :datatype-) "mas514/ServoSetpoints")
(setf (get mas514::ServoSetpoints :definition-)
      "int64 rightWheel
int64 leftWheel
int64 servo1
int64 servo2
int64 servo3
")



(provide :mas514/ServoSetpoints "790ff8777d41a4ebf31d8435d32b565b")


