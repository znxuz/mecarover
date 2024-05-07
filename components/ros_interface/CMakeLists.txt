idf_component_register(SRCS ros_interface.cc
                        INCLUDE_DIRS "." "../../config" "../../include"
                        REQUIRES mrlogger Controller micro_ros_espidf_component UltrasonicSensor
                    )

                    target_link_libraries(quiz_node ${catkin_LIBRARIES})