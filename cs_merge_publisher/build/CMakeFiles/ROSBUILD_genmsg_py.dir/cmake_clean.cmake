FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/cs_merge_controller/msg"
  "../src/cs_merge_controller/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/cs_merge_controller/msg/__init__.py"
  "../src/cs_merge_controller/msg/_transform.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
