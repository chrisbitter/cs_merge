FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/cs_merge_controller/msg"
  "../src/cs_merge_controller/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/cs_merge_controller/srv/__init__.py"
  "../src/cs_merge_controller/srv/_cs_hough.py"
  "../src/cs_merge_controller/srv/_getTransform.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
