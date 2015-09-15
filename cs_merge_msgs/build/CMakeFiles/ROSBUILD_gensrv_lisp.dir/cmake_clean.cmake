FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/cs_merge_msgs/msg"
  "../src/cs_merge_msgs/srv"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/getWorld.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_getWorld.lisp"
  "../srv_gen/lisp/getTransform.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_getTransform.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
