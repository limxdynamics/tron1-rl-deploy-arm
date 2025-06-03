cmake_minimum_required(VERSION 3.5)

project(airbot_sdk)

set(CMAKE_BUILD_TYPE "Release")

include_directories(${PROJECT_SOURCE_DIR}/include)

if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_VERSION} EQUAL 1)
    # In ROS compilation environment
    message("Currently in ROS1 compilation environment")
    
    find_package(catkin REQUIRED COMPONENTS 
      roscpp
    )

    catkin_package(
      INCLUDE_DIRS include
      CATKIN_DEPENDS
        roscpp
    )

    include_directories(${catkin_INCLUDE_DIRS})

    add_custom_target(airbot_sdk_include_target ALL
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/include ${CATKIN_DEVEL_PREFIX}/include
    )
    add_custom_target(airbot_sdk_target ALL
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/lib ${CATKIN_DEVEL_PREFIX}/lib
    )

    install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
    
    install(DIRECTORY lib/. DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION})

    install(DIRECTORY bin/. DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
      FILE_PERMISSIONS
      OWNER_READ OWNER_WRITE OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE
    )
  elseif($ENV{ROS_VERSION} EQUAL 2)
    # In ROS2 compilation environment
    message("Currently in ROS2 compilation environment")

    find_package(ament_cmake REQUIRED)

    install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include/)
    
    # Install library files to build output
    install(DIRECTORY lib/. DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

    install(DIRECTORY bin/. DESTINATION lib/${PROJECT_NAME}
      FILE_PERMISSIONS
      OWNER_READ OWNER_WRITE OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE
    )

    # Export the library
    ament_export_libraries(airbot_sdk)

    # Export the include
    ament_export_include_directories(include)

    ament_package()
  endif()
else()

  install(DIRECTORY ${PROJECT_SOURCE_DIR}/include DESTINATION ${CMAKE_INSTALL_PREFIX})

  # Install library files to build output
  install(DIRECTORY lib/. DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

  install(DIRECTORY bin/. DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
    FILE_PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
  )
endif()
