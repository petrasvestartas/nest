# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.29

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.29.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.29.2/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/petras/brg/2_code/nest

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/petras/brg/2_code/nest/build

# Utility rule file for clipper_2.

# Include any custom commands dependencies for this target.
include CMakeFiles/clipper_2.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/clipper_2.dir/progress.make

CMakeFiles/clipper_2: CMakeFiles/clipper_2-complete

CMakeFiles/clipper_2-complete: clipper_2-prefix/src/clipper_2-stamp/clipper_2-install
CMakeFiles/clipper_2-complete: clipper_2-prefix/src/clipper_2-stamp/clipper_2-mkdir
CMakeFiles/clipper_2-complete: clipper_2-prefix/src/clipper_2-stamp/clipper_2-download
CMakeFiles/clipper_2-complete: clipper_2-prefix/src/clipper_2-stamp/clipper_2-update
CMakeFiles/clipper_2-complete: clipper_2-prefix/src/clipper_2-stamp/clipper_2-patch
CMakeFiles/clipper_2-complete: clipper_2-prefix/src/clipper_2-stamp/clipper_2-configure
CMakeFiles/clipper_2-complete: clipper_2-prefix/src/clipper_2-stamp/clipper_2-build
CMakeFiles/clipper_2-complete: clipper_2-prefix/src/clipper_2-stamp/clipper_2-install
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/petras/brg/2_code/nest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'clipper_2'"
	/usr/local/Cellar/cmake/3.29.2/bin/cmake -E make_directory /Users/petras/brg/2_code/nest/build/CMakeFiles
	/usr/local/Cellar/cmake/3.29.2/bin/cmake -E touch /Users/petras/brg/2_code/nest/build/CMakeFiles/clipper_2-complete
	/usr/local/Cellar/cmake/3.29.2/bin/cmake -E touch /Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp/clipper_2-done

clipper_2-prefix/src/clipper_2-stamp/clipper_2-update:
.PHONY : clipper_2-prefix/src/clipper_2-stamp/clipper_2-update

clipper_2-prefix/src/clipper_2-stamp/clipper_2-build: clipper_2-prefix/src/clipper_2-stamp/clipper_2-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/petras/brg/2_code/nest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "No build step for 'clipper_2'"
	cd /Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-build && /usr/local/Cellar/cmake/3.29.2/bin/cmake -E echo_append
	cd /Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-build && /usr/local/Cellar/cmake/3.29.2/bin/cmake -E touch /Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp/clipper_2-build

clipper_2-prefix/src/clipper_2-stamp/clipper_2-configure: clipper_2-prefix/tmp/clipper_2-cfgcmd.txt
clipper_2-prefix/src/clipper_2-stamp/clipper_2-configure: clipper_2-prefix/src/clipper_2-stamp/clipper_2-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/petras/brg/2_code/nest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "No configure step for 'clipper_2'"
	cd /Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-build && /usr/local/Cellar/cmake/3.29.2/bin/cmake -E echo_append
	cd /Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-build && /usr/local/Cellar/cmake/3.29.2/bin/cmake -E touch /Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp/clipper_2-configure

clipper_2-prefix/src/clipper_2-stamp/clipper_2-download: clipper_2-prefix/src/clipper_2-stamp/clipper_2-gitinfo.txt
clipper_2-prefix/src/clipper_2-stamp/clipper_2-download: clipper_2-prefix/src/clipper_2-stamp/clipper_2-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/petras/brg/2_code/nest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (git clone) for 'clipper_2'"
	cd /Users/petras/brg/2_code/nest/build/install && /usr/local/Cellar/cmake/3.29.2/bin/cmake -P /Users/petras/brg/2_code/nest/build/clipper_2-prefix/tmp/clipper_2-gitclone.cmake
	cd /Users/petras/brg/2_code/nest/build/install && /usr/local/Cellar/cmake/3.29.2/bin/cmake -E touch /Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp/clipper_2-download

clipper_2-prefix/src/clipper_2-stamp/clipper_2-install: clipper_2-prefix/src/clipper_2-stamp/clipper_2-build
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/petras/brg/2_code/nest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "No install step for 'clipper_2'"
	cd /Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-build && /usr/local/Cellar/cmake/3.29.2/bin/cmake -E echo_append
	cd /Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-build && /usr/local/Cellar/cmake/3.29.2/bin/cmake -E touch /Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp/clipper_2-install

clipper_2-prefix/src/clipper_2-stamp/clipper_2-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/petras/brg/2_code/nest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Creating directories for 'clipper_2'"
	/usr/local/Cellar/cmake/3.29.2/bin/cmake -Dcfgdir= -P /Users/petras/brg/2_code/nest/build/clipper_2-prefix/tmp/clipper_2-mkdirs.cmake
	/usr/local/Cellar/cmake/3.29.2/bin/cmake -E touch /Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp/clipper_2-mkdir

clipper_2-prefix/src/clipper_2-stamp/clipper_2-patch: clipper_2-prefix/src/clipper_2-stamp/clipper_2-patch-info.txt
clipper_2-prefix/src/clipper_2-stamp/clipper_2-patch: clipper_2-prefix/src/clipper_2-stamp/clipper_2-update
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/petras/brg/2_code/nest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "No patch step for 'clipper_2'"
	/usr/local/Cellar/cmake/3.29.2/bin/cmake -E echo_append
	/usr/local/Cellar/cmake/3.29.2/bin/cmake -E touch /Users/petras/brg/2_code/nest/build/clipper_2-prefix/src/clipper_2-stamp/clipper_2-patch

clipper_2-prefix/src/clipper_2-stamp/clipper_2-update:
.PHONY : clipper_2-prefix/src/clipper_2-stamp/clipper_2-update

clipper_2-prefix/src/clipper_2-stamp/clipper_2-update: clipper_2-prefix/tmp/clipper_2-gitupdate.cmake
clipper_2-prefix/src/clipper_2-stamp/clipper_2-update: clipper_2-prefix/src/clipper_2-stamp/clipper_2-update-info.txt
clipper_2-prefix/src/clipper_2-stamp/clipper_2-update: clipper_2-prefix/src/clipper_2-stamp/clipper_2-download
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/petras/brg/2_code/nest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Performing update step for 'clipper_2'"
	cd /Users/petras/brg/2_code/nest/build/install/clipper_2 && /usr/local/Cellar/cmake/3.29.2/bin/cmake -Dcan_fetch=YES -P /Users/petras/brg/2_code/nest/build/clipper_2-prefix/tmp/clipper_2-gitupdate.cmake

clipper_2: CMakeFiles/clipper_2
clipper_2: CMakeFiles/clipper_2-complete
clipper_2: clipper_2-prefix/src/clipper_2-stamp/clipper_2-build
clipper_2: clipper_2-prefix/src/clipper_2-stamp/clipper_2-configure
clipper_2: clipper_2-prefix/src/clipper_2-stamp/clipper_2-download
clipper_2: clipper_2-prefix/src/clipper_2-stamp/clipper_2-install
clipper_2: clipper_2-prefix/src/clipper_2-stamp/clipper_2-mkdir
clipper_2: clipper_2-prefix/src/clipper_2-stamp/clipper_2-patch
clipper_2: clipper_2-prefix/src/clipper_2-stamp/clipper_2-update
clipper_2: CMakeFiles/clipper_2.dir/build.make
.PHONY : clipper_2

# Rule to build all files generated by this target.
CMakeFiles/clipper_2.dir/build: clipper_2
.PHONY : CMakeFiles/clipper_2.dir/build

CMakeFiles/clipper_2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/clipper_2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/clipper_2.dir/clean

CMakeFiles/clipper_2.dir/depend:
	cd /Users/petras/brg/2_code/nest/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/petras/brg/2_code/nest /Users/petras/brg/2_code/nest /Users/petras/brg/2_code/nest/build /Users/petras/brg/2_code/nest/build /Users/petras/brg/2_code/nest/build/CMakeFiles/clipper_2.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/clipper_2.dir/depend
