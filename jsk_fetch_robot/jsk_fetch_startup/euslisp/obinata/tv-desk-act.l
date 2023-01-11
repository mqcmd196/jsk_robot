#!/usr/bin/env roseus

(require :fetch-interface "package://fetcheus/fetch-interface.l")

(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")
(load "package://jsk_fetch_startup/euslisp/obinata/primitive-actions.l")

(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "fetch_anomaly_detection")
(ros::roseus "obinata_demo")

(unless (boundp '*ri*)
  (fetch-init))

(setq *spot-id* 2)

(setq *class-ac* (instance ros::simple-action-client :init
                           "/classification_server/cluster_server"
                           fetch_anomaly_detection::CheckClusterAction))
(setq *task-ac* (instance ros::simple-action-client :init
                          "/task_decision"
                           fetch_anomaly_detection::TaskDecisionAction))


(defun check-tv-desk-and-act()

  ;; TODO navigate to desk-front
  (go-to-spot "/eng2/7f/room73B2-tv-desk")
  (send *fetch* :angle-vector #f(21.5684 75.5612 80.1877 -11.4819 93.608 -0.091436 95.2235 -0.006954 -2.16963 19.5739))
  (send *ri* :angle-vector-raw (send *fetch* :angle-vector) 3000 :head-controller)
  (send *ri* :wait-interpolation)


  (setq *current-image* (one-shot-subscribe "/head_camera/rgb/image_rect_color" sensor_msgs::Image))

  ;; classification
  (setq *class-goal* (instance fetch_anomaly_detection::CheckClusterActionGoal :init))
  (send (send *class-goal* :goal) :image *current-image*)
  (send (send *class-goal* :goal) :spot_id *spot-id*)
  (send *class-ac* :send-goal *class-goal*)
  (send *class-ac* :wait-for-result)
  (setq *class-res* (send *class-ac* :get-result))

  ;; ask task to task server
  (setq *task-goal* (instance fetch_anomaly_detection::TaskDecisionActionGoal :init))
  (send (send *task-goal* :goal) :label (send *class-res* :label))
  (send (send *task-goal* :goal) :image *current-image*)
  (send (send *task-goal* :goal) :spot_id *spot-id*)
  (send *task-ac* :send-goal *task-goal*)
  (send *task-ac* :wait-for-result)
  (setq *task-res* (send *task-ac* :get-result))

  ;; execute task
  (setq *task-name* (send *task-res* :task_name))
  (cond
    ((equal *task-name* "tell-news")
     (tell-news))
    ((equal *task-name* "nothing")
     (ros::ros-info "Nothing to do"))
    ((equal *task-name* "tidy-up")
     (tidy-up))
    ((equal *task-name* "quiet")
     (quiet))
    )

  ;; Go anywhere
  ;; (go-to-spot "/eng2/7f/room73B2-kitchen-door-front")
  )

(check-tv-desk-and-act)
