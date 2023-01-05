;; AT Spot2 actions
;; Tell news
;; silent
;; notify

(load "package://fetcheus/fetch-interface.l")
(ros::roseus-add-msgs "newsapi_ros")
(ros::roseus-add-msgs "fetch_anomaly_detection")

(fetch-init)

(setq *news-client* (instance ros::simple-action-client :init
                              "/newsapi_ros/get_top_headlines"
                              newsapi_ros::GetTopHeadlinesAction))


(defun get-technology-news()
  ;; TODO add speak function
  ;; しゃべるならPC Watchか？
  ;; source はImpress.co.jp
  (let* ((goal (instance newsapi_ros::GetTopHeadlinesGoal :init))
         (speak-content ""))
    (send goal :category "technology")
    (send *news-client* :send-goal goal)
    (send *news-client* :wait-for-result)
    (setq *news* (send *news-client* :get-result))
    (setq speak-content (concatenate string speak-content "IT製品関連のニュースを話すよ。"))
    (dolist (n (send *news* :news_list))
      (if (string= (send n :source) "Itmedia.co.jp")
          (setq speak-content (concatenate string speak-content (send n :description)))
          nil)
      )
    (ros::ros-info (format nil "Spoke news ~A" speak-content))
    *news*
    )
  )

(defclass execute-primitive-actions
    :slots (header-file go-pos-as move-to-as say-as state-as take-a-picture-as))

(defmethod execute-primitive-actions
    (:init ()
     ;; The actions to be added
     ;; - get-technology-news
     ;; - silence
     ;; - tidy up
     (setq primitive-action-execute
           (instance ros::simple-action-server
                     :init "execute_primitive"
                     fetch_anomaly_detection::
                     :execute-cb `(lambda (s g) (send ,self :primitive s g))))
     self)

  ;; callbacks
  (:primitive (s g)
              (let* ((msg (send s :result))
                     (action-name (send (send g :goal :action_name))))
                (cond
                  ((= action-name "get_technology_news")
                   (get-technology-news)
                   (= action-name "silence")
                   (silence)))))

  (:spin ()
         (ros::rate 10)
         (while (ros::ok)
           (send go-pos-as :worker)
           (send move-to-as :worker)
           (send say-as :worker)
           (send take-a-picture-as :worker)
           (ros::spin-once)
           (ros::sleep))))

(defun main()
  (ros::roseus "primitive_actions")
  (setq *node* (instance execute-primitive-actions :init))
  (send *node* :spin))

(main)




;; (dolist (n (send *news* :news_list))
;;   (ros::ros-info (format nil "title ~A" (send n :title)))
;;   (ros::ros-info (format nil "source ~A" (send n :source)))
;;   )
