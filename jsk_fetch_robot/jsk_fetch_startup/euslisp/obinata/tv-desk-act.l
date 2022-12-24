#!/usr/bin/env roseus

(require :fetch-interface "package://fetcheus/fetch-interface.l")
(require :state-machine "package://roseus_smach/src/state-machine.l")
(require :state-machine-ros "package://roseus_smach/src/state-machine-ros.l")
(require :state-machine-utils "package://roseus_smach/src/state-machine-utils.l")
(require :base64 "lib/llib/base64.l")

(load "package://jsk_fetch_startup/euslisp/navigation-utils.l")

(ros::roseus-add-msgs "jsk_recognition_msgs")

;; *kitchen-class-result*
