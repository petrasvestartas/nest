########################################################################
# PROJECT SETUP
########################################################################
cmake_minimum_required(VERSION 3.24.2)
project(nest LANGUAGES C CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler configuration
message(STATUS "C++ compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "C++ compiler ID: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "C++ compiler version: ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "CMAKE_CXX_STANDARD: ${CMAKE_CXX_STANDARD}")

# Compiler specific messages
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR APPLE)
  message(STATUS "The C++ compiler is GCC.")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  message(STATUS "The C++ compiler is Microsoft Visual C++.")
else()
  message(STATUS "The C++ compiler is not GCC or Microsoft Visual C++.")
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")
set(CMAKE_BUILD_TYPE_INIT "Release")

# Policy configurations
cmake_policy(SET CMP0097 NEW)
cmake_policy(SET CMP0135 NEW)

########################################################################
# EXTERNAL PROJECTS
########################################################################
include(ExternalProject)
set(GET_LIBS "" CACHE STRING "Set option to download dependencies")
if(GET_LIBS)
  ExternalProject_Add(clipper_2
    GIT_REPOSITORY https://github.com/AngusJohnson/Clipper2.git
    GIT_TAG main
    CMAKE_ARGS
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    SOURCE_DIR "${CMAKE_BINARY_DIR}/install/clipper_2"
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
  )
endif()

########################################################################
# BUILD CONFIGURATIONS
########################################################################
set(BUILD_MY_PROJECTS "" CACHE STRING "Build Project")
if(BUILD_MY_PROJECTS)

  add_executable(${PROJECT_NAME} "${CMAKE_SOURCE_DIR}/src/main.cpp")
  
  list(APPEND headers "${CMAKE_SOURCE_DIR}/src/nest/" "${CMAKE_SOURCE_DIR}/src/sqlite/" "${CMAKE_BINARY_DIR}/install/clipper_2/CPP/Clipper2Lib/include")
  target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${headers}>") #header for the library

  

  target_precompile_headers(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src/stdafx.h)
  set(CMAKE_PCH_INSTANTIATE_TEMPLATES ON)


  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_link_libraries(${PROJECT_NAME} PUBLIC Clipper2 sqlite3)
  elseif(APPLE)
    target_link_directories(${PROJECT_NAME} PUBLIC "${CMAKE_BINARY_DIR}/")
    target_link_libraries(${PROJECT_NAME} PUBLIC Clipper2 sqlite3)
  else()
    target_link_directories(${PROJECT_NAME} PUBLIC "${CMAKE_BINARY_DIR}/Release/")
    target_link_libraries(${PROJECT_NAME} PUBLIC Clipper2 sqlite3)
  endif()
endif()

########################################################################
# LIBRARY COMPILATION
########################################################################
set(COMPILE_LIBS "" CACHE STRING "Set option to compile dependencies")
if(COMPILE_LIBS)
  set(CLIPPER2_INC
    "${CMAKE_BINARY_DIR}/install/clipper_2/CPP/Clipper2Lib/include/clipper2/clipper.core.h"
    "${CMAKE_BINARY_DIR}/install/clipper_2/CPP/Clipper2Lib/include/clipper2/clipper.engine.h"
    "${CMAKE_BINARY_DIR}/install/clipper_2/CPP/Clipper2Lib/include/clipper2/clipper.h"
    "${CMAKE_BINARY_DIR}/install/clipper_2/CPP/Clipper2Lib/include/clipper2/clipper.minkowski.h"
    "${CMAKE_BINARY_DIR}/install/clipper_2/CPP/Clipper2Lib/include/clipper2/clipper.offset.h"
  )
  set(CLIPPER2_SRC
    "${CMAKE_BINARY_DIR}/install/clipper_2/CPP/Clipper2Lib/src/clipper.engine.cpp"
    "${CMAKE_BINARY_DIR}/install/clipper_2/CPP/Clipper2Lib/src/clipper.offset.cpp"
  )
  add_library(Clipper2 STATIC ${CLIPPER2_INC} ${CLIPPER2_SRC})
  target_include_directories(Clipper2 PUBLIC "${CMAKE_BINARY_DIR}/install/clipper_2/CPP/Clipper2Lib/include/")
  
  add_library(sqlite3 STATIC "${CMAKE_SOURCE_DIR}/src/sqlite/sqlite3.c")
  target_include_directories(sqlite3 PUBLIC "${CMAKE_SOURCE_DIR}/src/sqlite/")
endif()

