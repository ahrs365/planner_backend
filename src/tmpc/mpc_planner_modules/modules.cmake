if(USE_ROS2)
	find_package(guidance_planner REQUIRED)
endif()
set(MODULE_DEPENDENCIES
	guidance_planner
)

set(MODULE_SOURCES
	src/mpc_base.cpp
	src/contouring.cpp
	src/guidance_constraints.cpp
	src/linearized_constraints.cpp
	src/ellipsoid_constraints.cpp
)
