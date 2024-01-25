;; Auto-generated. Do not edit!


(when (boundp 'barc::input)
  (if (not (find-package "BARC"))
    (make-package "BARC"))
  (shadow 'input (find-package "BARC")))
(unless (find-package "BARC::INPUT")
  (make-package "BARC::INPUT"))

(in-package "ROS")
;;//! \htmlinclude input.msg.html


(defclass barc::input
  :super ros::object
  :slots (_accel _steer ))

(defmethod barc::input
  (:init
   (&key
    ((:accel __accel) 0.0)
    ((:steer __steer) 0.0)
    )
   (send-super :init)
   (setq _accel (float __accel))
   (setq _steer (float __steer))
   self)
  (:accel
   (&optional __accel)
   (if __accel (setq _accel __accel)) _accel)
  (:steer
   (&optional __steer)
   (if __steer (setq _steer __steer)) _steer)
  (:serialization-length
   ()
   (+
    ;; float64 _accel
    8
    ;; float64 _steer
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _accel
       (sys::poke _accel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _steer
       (sys::poke _steer (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _accel
     (setq _accel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _steer
     (setq _steer (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get barc::input :md5sum-) "be360b2b21a0fff5bfe4192d75a591ef")
(setf (get barc::input :datatype-) "barc/input")
(setf (get barc::input :definition-)
      "float64 accel
float64 steer
")



(provide :barc/input "be360b2b21a0fff5bfe4192d75a591ef")


