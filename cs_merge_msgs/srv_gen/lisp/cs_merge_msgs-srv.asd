
(cl:in-package :asdf)

(defsystem "cs_merge_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :cs_merge_msgs-msg
               :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "getWorld" :depends-on ("_package_getWorld"))
    (:file "_package_getWorld" :depends-on ("_package"))
    (:file "getTransform" :depends-on ("_package_getTransform"))
    (:file "_package_getTransform" :depends-on ("_package"))
  ))