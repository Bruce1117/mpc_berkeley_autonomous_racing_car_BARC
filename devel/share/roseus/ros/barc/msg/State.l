;; Auto-generated. Do not edit!


(when (boundp 'barc::State)
  (if (not (find-package "BARC"))
    (make-package "BARC"))
  (shadow 'State (find-package "BARC")))
(unless (find-package "BARC::STATE")
  (make-package "BARC::STATE"))

(in-package "ROS")
;;//! \htmlinclude State.msg.html


(defclass barc::State
  :super ros::object
  :slots (_x _y _v _psi ))

(defmethod barc::State
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:v __v) 0.0)
    ((:psi __psi) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _v (float __v))
   (setq _psi (float __psi))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:psi
   (&optional __psi)
   (if __psi (setq _psi __psi)) _psi)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _v
    8
    ;; float64 _psi
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
     ;; float64 _v
       (sys::poke _v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _psi
       (sys::poke _psi (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v
     (setq _v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _psi
     (setq _psi (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get barc::State :md5sum-) "fe3ac2b76adaa1950be1785133b8a2c8")
(setf (get barc::State :datatype-) "barc/State")
(setf (get barc::State :definition-)
      "float64 x
float64 y
float64 v
float64 psi
")



(provide :barc/State "fe3ac2b76adaa1950be1785133b8a2c8")


