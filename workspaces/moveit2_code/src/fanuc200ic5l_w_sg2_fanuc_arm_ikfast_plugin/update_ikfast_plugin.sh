search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=fanuc200ic5l_w_sg2.srdf
robot_name_in_srdf=fanuc200ic5l_w_sg2
moveit_config_pkg=fanuc200ic5l_w_sg2_moveit_config
robot_name=fanuc200ic5l_w_sg2
planning_group_name=fanuc_arm
ikfast_plugin_pkg=fanuc200ic5l_w_sg2_fanuc_arm_ikfast_plugin
base_link_name=base_link
eef_link_name=tool0
ikfast_output_path=/root/catkin_ws/fanuc200ic5l_w_sg2_fanuc_arm_ikfast_plugin/src/fanuc200ic5l_w_sg2_fanuc_arm_ikfast_solver.cpp
eef_direction="0 0 1"

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  --eef_direction $eef_direction\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
