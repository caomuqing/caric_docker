# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /root/ws_caric/src/star_planner/nlopt-2.7.1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/ws_caric/src/star_planner/nlopt-2.7.1/build

# Include any dependencies generated for this target.
include test/CMakeFiles/testopt.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/testopt.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/testopt.dir/flags.make

test/CMakeFiles/testopt.dir/testfuncs.c.o: test/CMakeFiles/testopt.dir/flags.make
test/CMakeFiles/testopt.dir/testfuncs.c.o: ../test/testfuncs.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/ws_caric/src/star_planner/nlopt-2.7.1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object test/CMakeFiles/testopt.dir/testfuncs.c.o"
	cd /root/ws_caric/src/star_planner/nlopt-2.7.1/build/test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/testopt.dir/testfuncs.c.o   -c /root/ws_caric/src/star_planner/nlopt-2.7.1/test/testfuncs.c

test/CMakeFiles/testopt.dir/testfuncs.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/testopt.dir/testfuncs.c.i"
	cd /root/ws_caric/src/star_planner/nlopt-2.7.1/build/test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /root/ws_caric/src/star_planner/nlopt-2.7.1/test/testfuncs.c > CMakeFiles/testopt.dir/testfuncs.c.i

test/CMakeFiles/testopt.dir/testfuncs.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/testopt.dir/testfuncs.c.s"
	cd /root/ws_caric/src/star_planner/nlopt-2.7.1/build/test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /root/ws_caric/src/star_planner/nlopt-2.7.1/test/testfuncs.c -o CMakeFiles/testopt.dir/testfuncs.c.s

test/CMakeFiles/testopt.dir/testopt.c.o: test/CMakeFiles/testopt.dir/flags.make
test/CMakeFiles/testopt.dir/testopt.c.o: ../test/testopt.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/ws_caric/src/star_planner/nlopt-2.7.1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object test/CMakeFiles/testopt.dir/testopt.c.o"
	cd /root/ws_caric/src/star_planner/nlopt-2.7.1/build/test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/testopt.dir/testopt.c.o   -c /root/ws_caric/src/star_planner/nlopt-2.7.1/test/testopt.c

test/CMakeFiles/testopt.dir/testopt.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/testopt.dir/testopt.c.i"
	cd /root/ws_caric/src/star_planner/nlopt-2.7.1/build/test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /root/ws_caric/src/star_planner/nlopt-2.7.1/test/testopt.c > CMakeFiles/testopt.dir/testopt.c.i

test/CMakeFiles/testopt.dir/testopt.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/testopt.dir/testopt.c.s"
	cd /root/ws_caric/src/star_planner/nlopt-2.7.1/build/test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /root/ws_caric/src/star_planner/nlopt-2.7.1/test/testopt.c -o CMakeFiles/testopt.dir/testopt.c.s

test/CMakeFiles/testopt.dir/__/src/util/timer.c.o: test/CMakeFiles/testopt.dir/flags.make
test/CMakeFiles/testopt.dir/__/src/util/timer.c.o: ../src/util/timer.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/ws_caric/src/star_planner/nlopt-2.7.1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object test/CMakeFiles/testopt.dir/__/src/util/timer.c.o"
	cd /root/ws_caric/src/star_planner/nlopt-2.7.1/build/test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/testopt.dir/__/src/util/timer.c.o   -c /root/ws_caric/src/star_planner/nlopt-2.7.1/src/util/timer.c

test/CMakeFiles/testopt.dir/__/src/util/timer.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/testopt.dir/__/src/util/timer.c.i"
	cd /root/ws_caric/src/star_planner/nlopt-2.7.1/build/test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /root/ws_caric/src/star_planner/nlopt-2.7.1/src/util/timer.c > CMakeFiles/testopt.dir/__/src/util/timer.c.i

test/CMakeFiles/testopt.dir/__/src/util/timer.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/testopt.dir/__/src/util/timer.c.s"
	cd /root/ws_caric/src/star_planner/nlopt-2.7.1/build/test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /root/ws_caric/src/star_planner/nlopt-2.7.1/src/util/timer.c -o CMakeFiles/testopt.dir/__/src/util/timer.c.s

test/CMakeFiles/testopt.dir/__/src/util/mt19937ar.c.o: test/CMakeFiles/testopt.dir/flags.make
test/CMakeFiles/testopt.dir/__/src/util/mt19937ar.c.o: ../src/util/mt19937ar.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/ws_caric/src/star_planner/nlopt-2.7.1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object test/CMakeFiles/testopt.dir/__/src/util/mt19937ar.c.o"
	cd /root/ws_caric/src/star_planner/nlopt-2.7.1/build/test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/testopt.dir/__/src/util/mt19937ar.c.o   -c /root/ws_caric/src/star_planner/nlopt-2.7.1/src/util/mt19937ar.c

test/CMakeFiles/testopt.dir/__/src/util/mt19937ar.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/testopt.dir/__/src/util/mt19937ar.c.i"
	cd /root/ws_caric/src/star_planner/nlopt-2.7.1/build/test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /root/ws_caric/src/star_planner/nlopt-2.7.1/src/util/mt19937ar.c > CMakeFiles/testopt.dir/__/src/util/mt19937ar.c.i

test/CMakeFiles/testopt.dir/__/src/util/mt19937ar.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/testopt.dir/__/src/util/mt19937ar.c.s"
	cd /root/ws_caric/src/star_planner/nlopt-2.7.1/build/test && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /root/ws_caric/src/star_planner/nlopt-2.7.1/src/util/mt19937ar.c -o CMakeFiles/testopt.dir/__/src/util/mt19937ar.c.s

# Object files for target testopt
testopt_OBJECTS = \
"CMakeFiles/testopt.dir/testfuncs.c.o" \
"CMakeFiles/testopt.dir/testopt.c.o" \
"CMakeFiles/testopt.dir/__/src/util/timer.c.o" \
"CMakeFiles/testopt.dir/__/src/util/mt19937ar.c.o"

# External object files for target testopt
testopt_EXTERNAL_OBJECTS =

test/testopt: test/CMakeFiles/testopt.dir/testfuncs.c.o
test/testopt: test/CMakeFiles/testopt.dir/testopt.c.o
test/testopt: test/CMakeFiles/testopt.dir/__/src/util/timer.c.o
test/testopt: test/CMakeFiles/testopt.dir/__/src/util/mt19937ar.c.o
test/testopt: test/CMakeFiles/testopt.dir/build.make
test/testopt: libnlopt.so.0.11.1
test/testopt: test/CMakeFiles/testopt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/ws_caric/src/star_planner/nlopt-2.7.1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable testopt"
	cd /root/ws_caric/src/star_planner/nlopt-2.7.1/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testopt.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/testopt.dir/build: test/testopt

.PHONY : test/CMakeFiles/testopt.dir/build

test/CMakeFiles/testopt.dir/clean:
	cd /root/ws_caric/src/star_planner/nlopt-2.7.1/build/test && $(CMAKE_COMMAND) -P CMakeFiles/testopt.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/testopt.dir/clean

test/CMakeFiles/testopt.dir/depend:
	cd /root/ws_caric/src/star_planner/nlopt-2.7.1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/ws_caric/src/star_planner/nlopt-2.7.1 /root/ws_caric/src/star_planner/nlopt-2.7.1/test /root/ws_caric/src/star_planner/nlopt-2.7.1/build /root/ws_caric/src/star_planner/nlopt-2.7.1/build/test /root/ws_caric/src/star_planner/nlopt-2.7.1/build/test/CMakeFiles/testopt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/testopt.dir/depend

