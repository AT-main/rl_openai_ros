execute_process(COMMAND "/home/amir/Documents/uni2/catkin_ws/build/openai_ros/openai_ros/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/amir/Documents/uni2/catkin_ws/build/openai_ros/openai_ros/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
