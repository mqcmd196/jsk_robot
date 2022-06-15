#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(ros::roseus "emotion-lister")
(load "package://unitreeeus/unitree-interface.l")

(defun chatting-cb(msg)
  (print "callback chatting-cb called")
  (progn (setq *speech* (send msg :data))
	 (print *speech*)
	 (cond ((string= *speech* "happy")
		(load "happy.l")
		(happy-main))
	       ((string=  *speech* "joy")
		(load "joy.l")
		(joy-main))
	       ((string= *speech* "affirmation")
		(load "affirmation.l")
		(affirmation-main))
	       ((string= *speech* "negation")
		(load "negation.l")
		(negation-main))
	       ((string= *speech* "love")
		(load "love.l")
		(love-main))
	       ((string= *speech* "scared")
		(load "scared.l")
		(scared-main))
	       (t
		(print "odd")))
	 (print "========================")
	 *speech*))
	       


(defun main()
  (progn (go1-init)
    (ros::subscribe "/emotion_chatter" std_msgs::String #'chatting-cb)
         (ros::rate 10)
         (while (ros::ok)
           (ros::spin-once))))
