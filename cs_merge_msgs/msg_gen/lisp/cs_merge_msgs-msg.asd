
(cl:in-package :asdf)

(defsystem "cs_merge_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "transform" :depends-on ("_package_transform"))
    (:file "_package_transform" :depends-on ("_package"))
  ))