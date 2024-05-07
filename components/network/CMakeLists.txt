set(COMPONENT_SRCS "imsl_network.cc")
set(COMPONENT_ADD_INCLUDEDIRS "." "$ENV{PROJECT_PATH}/include" "$ENV{PROJECT_PATH}/config")
set(COMPONENT_REQUIRES arduino mrlogger)
register_component()
