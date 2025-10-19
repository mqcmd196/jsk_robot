#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")
(unless (boundp '*spot*) (spot) (send *spot* :reset-pose))
(unless (ros::ok) (ros::roseus "spot_eus_interface"))
(unless (boundp '*ri*) (setq *ri* (instance spot-interface :init)))
(send *ri* :claim)

(setq ip (unix:getenv "ROS_IP"))

(setq info-txt (format nil "こんにちは, 私の名前は~Aです．私のインターネットアドレスは~A, 私のオーナーは~A, 私のドックIDは~Aです．"
                       (ros::get-param "/robot/name")
                       ip
                       (unix::getenv "USER")
                       (round (ros::get-param "/spot/spot_ros/dock_id"))))
(ros::ros-info info-txt)
(send *ri* :speak-jp info-txt :wait t)


(ros::exit)
