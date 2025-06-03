

include(CMakeFindDependencyMacro)
include("${CMAKE_CURRENT_LIST_DIR}/airbot_playTargets.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/airbot_playVersion.cmake")

include(CMakeFindDependencyMacro)
find_dependency(spdlog REQUIRED)
find_dependency(fmt REQUIRED)
find_dependency(Eigen3 REQUIRED)
find_dependency(ZLIB REQUIRED)
find_dependency(Boost COMPONENTS system)
find_dependency(console_bridge REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
