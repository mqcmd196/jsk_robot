#!/usr/bin/env roseus

(load "package://jsk_robot_startup/lifelog/tweet_client.l")

(tweet-string "I took a photo." :warning-time 3 :with-image t :speak t)

(unix::system (format nil "rosrun dialogflow_task_executive save_dialogflow_json_response.py -a Tweet -t \"写真を撮ってツイートしました\" -i http://webvideo.fetch1075.jsk.imi.i.u-tokyo.ac.jp/snapshot?topic=/head_camera/rgb/throttled/image_rect_color"))
(exit)
