cmake_minimum_required(VERSION 3.5.1)
project(lanelet2)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-backtrace-limit=0")
# by setting this, we don't need to install the generated c++ library to /usr/local
# LINKED_LIBRARIES will be installed by side with the python api binding libraries
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath='$ORIGIN'")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
option(USE_PYTHON3 "Use python 3 to build libraries." ON)
if (USE_PYTHON3)
  find_package(Boost REQUIRED COMPONENTS python-py35 serialization system filesystem)
  find_package(PythonLibs 3.5 REQUIRED) # sets ${PYTHON_INCLUDE_DIRS}
else()
  find_package(Boost REQUIRED COMPONENTS python serialization system filesystem)
  find_package(PythonLibs 2.7 REQUIRED) # sets ${PYTHON_INCLUDE_DIRS}
endif()
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(EIGEN3 REQUIRED eigen3)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()
find_package(GeographicLib REQUIRED)
find_package(PugiXML REQUIRED)

include_directories(
  lanelet2_core/include
  lanelet2_io/include
  lanelet2_projection/include
  lanelet2_routing/include
  lanelet2_traffic_rules/include
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GeographicLib_INCLUDE_DIRS}
  ${PugiXML_INCLUDE_DIRS}
)

set(MODUELS core io projection routing traffic_rules geometry)

set(LINKED_LIBRARIES "")
# build the c++ libraries first
foreach(SUBMODULE ${MODUELS})
  if (NOT SUBMODULE STREQUAL "geometry")
    set(LIBRARY_NAME ${PROJECT_NAME}_${SUBMODULE})
    set(SRCS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_${SUBMODULE}/src)
    file(GLOB ${SUBMODULE}_SRC ${SRCS_DIR}/*.cpp)
    add_library(${LIBRARY_NAME} SHARED
      ${${SUBMODULE}_SRC}
    )
    target_include_directories(${LIBRARY_NAME}
      PUBLIC   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
      PRIVATE  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}_${SUBMODULE}/include/${PROJECT_NAME}_${SUBMODULE}>)
    target_link_libraries(${LIBRARY_NAME}
      ${Boost_LIBRARIES}
      ${PYTHON_LIBRARY}
      ${GeographicLib_LIBRARIES}
      ${PugiXML_LIBRARIES}
    )
    list(APPEND LINKED_LIBRARIES ${LIBRARY_NAME})
    install(TARGETS ${LIBRARY_NAME}
      ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
      LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(DIRECTORY ${PROJECT_NAME}_${SUBMODULE}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
  endif()
endforeach()


# build each python api binding submodule
foreach(SUBMODULE ${MODUELS})
   set(API_LIBRARY_NAME "${PROJECT_NAME}_${SUBMODULE}_pyapi")
   add_library(${API_LIBRARY_NAME} SHARED
     lanelet2_python/python_api/${SUBMODULE}.cpp  # binding codes
   )
   target_compile_definitions(${API_LIBRARY_NAME} PRIVATE -DPYTHON_API_MODULE_NAME=lib${API_LIBRARY_NAME})
   target_link_libraries(${API_LIBRARY_NAME}
     ${Boost_LIBRARIES}
     ${PYTHON_LIBRARY}
     ${LINKED_LIBRARIES}
   )
endforeach()
