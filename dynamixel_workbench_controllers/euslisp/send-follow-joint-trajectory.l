;; Usage
;; (if needed) overwrite euslib/config/yamaguchi_dynamixel.yaml
;; roslaunch euslib dynamixel_controllers.launch
;; (angle-vector)              ;; get current angle of dynamixel
;; (angle-vector #f(100) 1000) ;; set angle of dynamixel to 100 [deg] by 1000 [ms]

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "control_msgs")
(ros::load-ros-manifest "trajectory_msgs")
(ros::load-ros-manifest "sensor_msgs")

(ros::roseus "send_follow_joint_trajectory")
(ros::advertise "/dynamixel_workbench/follow_joint_trajectory_action/goal" control_msgs::FollowJointTrajectoryActionGoal 1)
(setq *joint-states* (one-shot-subscribe "/dynamixel_workbench/joint_states" sensor_msgs::JointState))
(setq *joint-names* (send *joint-states* :name))

(defun angle-vector (&optional (av nil) (time 3000))
  (let ((follow-joint-trajectory-goal (instance control_msgs::FollowJointTrajectoryActionGoal :init))
        (joint-trajectory-point (instance trajectory_msgs::JointTrajectoryPoint :init))
        (ros-time (send (ros::time) :now) :sec))
    ;; if av is not set, return current angle-vector
    (unless av
      (setq *joint-states* (one-shot-subscribe "/dynamixel_workbench/joint_states" sensor_msgs::JointState))
      (return-from angle-vector
        (map float-vector #'rad2deg (send *joint-states* :position))))
    ;; check length of angle-vector
    (when (not (eq (length av) (length *joint-names*)))
      (format t "length of angle-vector does not equal to length of joint names~%")
      (return-from angle-vector nil))
    ;; change av of dynamixel
    (send joint-trajectory-point :positions (map float-vector #'deg2rad av))
    (send joint-trajectory-point :velocities
          (coerce (make-list (length av) :initial-element 0) float-vector))
    (send (send joint-trajectory-point :time_from_start) :sec (/ time 1000))
    (send (send joint-trajectory-point :time_from_start) :nsec (- time (* (/ time 1000) 1000)))
    (send follow-joint-trajectory-goal :goal :trajectory :points (list joint-trajectory-point))
    (send (send follow-joint-trajectory-goal :goal :trajectory :header :stamp)
          :sec (send ros-time :sec))
    (send (send follow-joint-trajectory-goal :goal :trajectory :header :stamp)
          :nsec (send ros-time :nsec))
    (send follow-joint-trajectory-goal :header :seq 1)
    (send (send follow-joint-trajectory-goal :header :stamp) :sec (send ros-time :sec))
    (send (send follow-joint-trajectory-goal :header :stamp) :nsec (send ros-time :nsec))
    (send follow-joint-trajectory-goal :goal :trajectory :joint_names *joint-names*)
    (ros::publish "/dynamixel_workbench/follow_joint_trajectory_action/goal" follow-joint-trajectory-goal)
    ))

(format t "Joint names:~%")
(dolist (jn *joint-names*)
  (format t "~A~%" jn))
(format t "Current angle-vector:~%")
(print (angle-vector))
