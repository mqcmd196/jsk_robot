#!/usr/bin/env roseus

(ros::roseus "speak_volume_lower")
(ros::set-dynamic-reconfigure-param "/audible_warning" "enable" :bool t)
(ros::set-dynamic-reconfigure-param "/tweet_client_tablet" "speak_enable" :bool t)
(ros::set-dynamic-reconfigure-param "/tweet_client_uptime" "speak_enable" :bool t)
(ros::set-dynamic-reconfigure-param "/tweet_client_warning" "speak_enable" :bool t)
(ros::set-dynamic-reconfigure-param "/tweet_client_worktime" "speak_enable" :bool t)
(ros::set-dynamic-reconfigure-param "/audible_warning" "volume" :double 0.2)
(ros::set-dynamic-reconfigure-param "/tweet_client_tablet" "volume" :double 0.2)
(ros::set-dynamic-reconfigure-param "/tweet_client_uptime" "volume" :double 0.2)
(ros::set-dynamic-reconfigure-param "/tweet_client_warning" "volume" :double 0.2)
(ros::set-dynamic-reconfigure-param "/tweet_client_worktime" "volume" :double 0.2)
(exit)