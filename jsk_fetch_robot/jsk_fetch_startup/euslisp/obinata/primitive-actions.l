;; AT Spot2 actions
;; Tell news
;; silent
;; notify

(unless (boundp '*ri*)
  (require :fetch-interface "package://fetcheus/fetch-interface.l")
  (fetch-init))

(ros::roseus-add-msgs "newsapi_ros")

(setq *news-client* (instance ros::simple-action-client :init
                              "/newsapi_ros/get_top_headlines"
                              newsapi_ros::GetTopHeadlinesAction))

(defun tell-news()
  ;; source はImpress.co.jp
  (let* ((goal (instance newsapi_ros::GetTopHeadlinesGoal :init))
         (speak-content ""))
    (send goal :category "technology")
    (send *news-client* :send-goal goal)
    (send *news-client* :wait-for-result)
    (setq *news* (send *news-client* :get-result))
    (setq speak-content (concatenate string speak-content "IT関連のニュースを話すよ。"))
    (dolist (n (send *news* :news_list))
      (if (string= (send n :source) "Itmedia.co.jp")
          (setq speak-content (concatenate string speak-content (send n :description)))

          nil))
    (ros::ros-info (format nil "Spoke news ~A" speak-content))
    (send *ri* :speak-jp speak-content :wait t)))

(defun quiet()
  (send *ri* :speak-jp "うるさいですね，静かにします" :wait t)
  (ros::set-dynamic-reconfigure-param "/audible_warning" "enable" :bool nil)
  (ros::set-dynamic-reconfigure-param "/tweet_client_tablet" "speak_enable" :bool nil)
  (ros::set-dynamic-reconfigure-param "/tweet_client_uptime" "speak_enable" :bool nil)
  (ros::set-dynamic-reconfigure-param "/tweet_client_warning" "speak_enable" :bool nil)
  (ros::set-dynamic-reconfigure-param "/tweet_client_worktime" "speak_enable" :bool nil)
  (ros::set-dynamic-reconfigure-param "/audible_warning" "volume" :double 0.0)
  (ros::set-dynamic-reconfigure-param "/tweet_client_tablet" "volume" :double 0.0)
  (ros::set-dynamic-reconfigure-param "/tweet_client_uptime" "volume" :double 0.0)
  (ros::set-dynamic-reconfigure-param "/tweet_client_warning" "volume" :double 0.0)
  (ros::set-dynamic-reconfigure-param "/tweet_client_worktime" "volume" :double 0.0))
