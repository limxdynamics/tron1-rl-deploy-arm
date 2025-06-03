cmake_minimum_required(VERSION 3.10)
project(airbot_msgs)

find_package(catkin REQUIRED std_msgs geometry_msgs sensor_msgs message_generation)

set(MsgFiles "")
FILE(GLOB_RECURSE MsgFilesPath "msg/*.msg")
FOREACH(FILE_PATH ${MsgFilesPath})
    STRING(REGEX REPLACE ".+/(.+)" "\\1" FILE_NAME ${FILE_PATH})
    list(APPEND MsgFiles ${FILE_NAME})
ENDFOREACH(FILE_PATH)

list(LENGTH MsgFiles MsgFilesLength)
IF(NOT ${MsgFilesLength} EQUAL "0")
  add_message_files(
    FILES
    ${MsgFiles}
  )
ENDIF()

FILE(GLOB_RECURSE SrvFilesPath "srv/*.srv")
FOREACH(FILE_PATH ${SrvFilesPath})
    STRING(REGEX REPLACE ".+/(.+)" "\\1" FILE_NAME ${FILE_PATH})
    list(APPEND SrvFiles ${FILE_NAME})
ENDFOREACH(FILE_PATH)

list(LENGTH SrvFiles SrvFilesLength)
IF(NOT ${SrvFilesLength} EQUAL "0")
  add_service_files(
    FILES
    ${SrvFiles}
  )
ENDIF()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package()
