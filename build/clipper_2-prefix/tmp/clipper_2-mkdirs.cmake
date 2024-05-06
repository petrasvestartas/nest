# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/petras/brg/2_code/nest/build/install/clipper_2"
  "/Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-build"
  "/Users/petras/brg/2_code/nest/build/clipper_2-prefix"
  "/Users/petras/brg/2_code/nest/build/clipper_2-prefix/tmp"
  "/Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp"
  "/Users/petras/brg/2_code/nest/build/clipper_2-prefix/src"
  "/Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp${cfgdir}") # cfgdir has leading slash
endif()
