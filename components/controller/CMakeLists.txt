idf_component_register(SRCS ControllerTasks.cc 
                        INCLUDE_DIRS "." "../../config" "../../include" 
                        REQUIRES mrlogger Eigen freertos HAL
                    )