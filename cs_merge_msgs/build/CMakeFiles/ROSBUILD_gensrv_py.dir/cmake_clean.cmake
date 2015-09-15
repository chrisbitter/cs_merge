FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/cs_merge_msgs/msg"
  "../src/cs_merge_msgs/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/cs_merge_msgs/srv/__init__.py"
  "../src/cs_merge_msgs/srv/_getWorld.py"
  "../src/cs_merge_msgs/srv/_getTransform.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
