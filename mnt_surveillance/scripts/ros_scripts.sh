# In this shell script we implement functions for running ros2 humble and 
# related ros workspaces

function source_ros_humble(){
  source /opt/ros/humble/setup.bash

  # Colcon CLI autocomplete
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
}

function source_turtlebot3_env(){
  source_ros_humble

  # Turtlebot exports
  export ROS_DOMAIN_ID=30 #TURTLEBOT3
  export LDS_MODEL=LDS-01
  export OPENCR_MODEL=burger
  export TURTLEBOT3_MODEL=burger

  # Source turtlebot workspace
  source ~/ros_ws/turtlebot3_ws/install/setup.bash
}

source_mentee_project_env(){
  source_ros_humble
  source ~/ros_ws/mentee_project_ws/install/setup.bash
}

function ros2_build(){
  colcon build --symlink-install 
  source install/setup.bash
}

function ros2_clean(){
  rm -rf build/ install/ log/ 
}

###########################################################
###################### script to run ######################
###########################################################
source_mentee_project_env
