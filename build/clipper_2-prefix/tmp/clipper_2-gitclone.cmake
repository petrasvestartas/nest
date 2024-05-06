# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

if(EXISTS "/Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp/clipper_2-gitclone-lastrun.txt" AND EXISTS "/Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp/clipper_2-gitinfo.txt" AND
  "/Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp/clipper_2-gitclone-lastrun.txt" IS_NEWER_THAN "/Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp/clipper_2-gitinfo.txt")
  message(STATUS
    "Avoiding repeated git clone, stamp file is up to date: "
    "'/Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp/clipper_2-gitclone-lastrun.txt'"
  )
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "/Users/petras/brg/2_code/nest/build/install/clipper_2"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/Users/petras/brg/2_code/nest/build/install/clipper_2'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/local/bin/git"
            clone --no-checkout --config "advice.detachedHead=false" "https://github.com/AngusJohnson/Clipper2.git" "clipper_2"
    WORKING_DIRECTORY "/Users/petras/brg/2_code/nest/build/install"
    RESULT_VARIABLE error_code
  )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once: ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/AngusJohnson/Clipper2.git'")
endif()

execute_process(
  COMMAND "/usr/local/bin/git"
          checkout "main" --
  WORKING_DIRECTORY "/Users/petras/brg/2_code/nest/build/install/clipper_2"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'main'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/local/bin/git" 
            submodule update --recursive --init 
    WORKING_DIRECTORY "/Users/petras/brg/2_code/nest/build/install/clipper_2"
    RESULT_VARIABLE error_code
  )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/Users/petras/brg/2_code/nest/build/install/clipper_2'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy "/Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp/clipper_2-gitinfo.txt" "/Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp/clipper_2-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp/clipper_2-gitclone-lastrun.txt'")
endif()
