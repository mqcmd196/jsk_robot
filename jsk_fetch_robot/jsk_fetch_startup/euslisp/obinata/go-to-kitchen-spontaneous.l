;; AT Spot2 actions
;; Tell news
;; silent
;; notify

(ros::roseus-add-msgs "newsapi_ros")


(setq *news-client* (instance ros::simple-action-client :init
                              "/newsapi_ros/get_top_headlines"
                              newsapi_ros::GetTopHeadlinesAction))

(defun get-technology-news()
  (let* ((goal (instance newsapi_ros::GetTopHeadlinesGoal :init))
         news)
    (send goal :category "technology")
    (send *news-client* :send-goal goal)
    (send *news-client* :wait-for-result)
    (setq news (send *news-client* :get-result))
    (dolist (n (send news :news_list))
      (ros::ros-info (format nil "News title ~A" (send n :title))))
    news
    )
)

(dolist (n (send *news* :news_list))
  (ros::ros-info (format nil "title ~A" (send n :title)))
  (ros::ros-info (format nil "source ~A" (send n :source)))
  )
