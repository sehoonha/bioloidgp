cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")

message(STATUS "=====================================================")
message(STATUS "configuring build for the simple balancing simulation")
message(STATUS "=====================================================")

#####################################################################
#           Explicit macro setup for debug configuration
#####################################################################
# enable or disable debugging, default is Release
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release")
ENDIF(NOT CMAKE_BUILD_TYPE)

message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

IF(CMAKE_BUILD_TYPE MATCHES "Release")
  message(STATUS "Set the release mode")
  ADD_DEFINITIONS(-DNDEBUG )
  ADD_DEFINITIONS(-DBOOST_UBLAS_NDEBUG )
ENDIF (CMAKE_BUILD_TYPE MATCHES "Release")

project(bioloidgp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-enum-compare")
message(STATUS "CXX_FLAGS = ${CMAKE_CXX_FLAGS}")


file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/glog")
message(STATUS "Create ${CMAKE_BINARY_DIR}/glog directory")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/captures")
message(STATUS "Create ${CMAKE_BINARY_DIR}/captures directory")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/captures")
# message(STATUS "Copy ${CMAKE_SOURCE_DIR}/data ${CMAKE_BINARY_DIR}")
# file(COPY ${CMAKE_SOURCE_DIR}/data DESTINATION ${CMAKE_BINARY_DIR})


message(STATUS "=====================================================")
message(STATUS "configuring dependancies...")
message(STATUS "=====================================================")

find_package(OpenGL)
if (OPENGL_FOUND)
  message(STATUS "OPENGL FOUND")
else (OPENGL_FOUND)
  message(STATUS "OPENGL NOT FOUND")
endif ()

find_package(GLUT)
if (GLUT_FOUND)
  message(STATUS "GLUT FOUND")
  message(STATUS "GLUT_LIBRARIES = ${GLUT_LIBRARIES}")
else (GLUT_FOUND)
  message(STATUS "GLUT NOT FOUND")
endif ()

find_package( Boost COMPONENTS date_time system filesystem thread chrono log log_setup program_options signals serialization)
message(STATUS "Boost_INCLUDE_DIR =  ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARIES =  ${Boost_LIBRARIES}")
include_directories(${Boost_INCLUDE_DIRS})

find_package(Qt4 4.6.0 REQUIRED QtCore QtGui QtXml QtOpenGL)
include(${QT_USE_FILE})
message(STATUS "QT_USE_FILE    = ${QT_USE_FILE}")
message(STATUS "QT_INCLUDES    = ${QT_INCLUDES}")
message(STATUS "QT_LIBRARIES   = ${QT_LIBRARIES}")

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(Eigen REQUIRED)
message(STATUS "Eigen_INCLUDE_DIRS = ${Eigen_INCLUDE_DIRS}")
include_directories(${Eigen_INCLUDE_DIRS})
find_package(TinyXML2 REQUIRED)
message(STATUS "TinyXML2_INCLUDE_DIRS = ${TinyXML2_INCLUDE_DIRS}")
message(STATUS "TinyXML2_LIBRARIES = ${TinyXML2_LIBRARIES}")

find_package(Bullet COMPONENTS BulletMath BulletCollision)
message(STATUS "BULLET_INCLUDE_DIRS = ${BULLET_INCLUDE_DIRS}")
include_directories(${BULLET_INCLUDE_DIRS})

message(STATUS "=====================================================")
message(STATUS "configuring the project")
message(STATUS "=====================================================")

include_directories(".")

file(GLOB bioloidgp_util_srcs "util/*.cpp")
file(GLOB bioloidgp_util_hdrs "util/*.h")
file(GLOB bioloidgp_robot_srcs "robot/*.cpp")
file(GLOB bioloidgp_robot_hdrs "robot/*.h")

# qt4_wrap_cpp(bioloidgp_moc_files app/Window.h app/GLWidget.h)
# message(STATUS "qt4_wrap_cpp expands the moc files: ${bioloidgp_moc_files}")

add_executable(bioloidgp
  ${bioloidgp_util_srcs} ${bioloidgp_util_hdrs}
  ${bioloidgp_robot_srcs} ${bioloidgp_robot_hdrs}
  # ${bioloidgp_moc_files}
  )

target_link_libraries(bioloidgp ${OPENGL_LIBRARIES})
target_link_libraries(bioloidgp ${GLUT_LIBRARIES})
target_link_libraries(bioloidgp ${QT_LIBRARIES})
target_link_libraries(bioloidgp ${Boost_LIBRARIES})
target_link_libraries(bioloidgp glog)

target_link_libraries(bioloidgp ${TinyXML2_LIBRARIES})
target_link_libraries(bioloidgp dart4 dart-core4)

set_target_properties(bioloidgp PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

