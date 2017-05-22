# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /home/andreanistico/Work/GazeboPlugins

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/andreanistico/Work/GazeboPlugins

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	cd /home/andreanistico/Work/GazeboPlugins && $(CMAKE_COMMAND) -E cmake_progress_start /home/andreanistico/Work/GazeboPlugins/CMakeFiles /home/andreanistico/Work/GazeboPlugins/landingPlat/CMakeFiles/progress.marks
	cd /home/andreanistico/Work/GazeboPlugins && $(MAKE) -f CMakeFiles/Makefile2 landingPlat/all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/andreanistico/Work/GazeboPlugins/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	cd /home/andreanistico/Work/GazeboPlugins && $(MAKE) -f CMakeFiles/Makefile2 landingPlat/clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	cd /home/andreanistico/Work/GazeboPlugins && $(MAKE) -f CMakeFiles/Makefile2 landingPlat/preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	cd /home/andreanistico/Work/GazeboPlugins && $(MAKE) -f CMakeFiles/Makefile2 landingPlat/preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	cd /home/andreanistico/Work/GazeboPlugins && $(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

# Convenience name for target.
landingPlat/CMakeFiles/platformPlugin.dir/rule:
	cd /home/andreanistico/Work/GazeboPlugins && $(MAKE) -f CMakeFiles/Makefile2 landingPlat/CMakeFiles/platformPlugin.dir/rule
.PHONY : landingPlat/CMakeFiles/platformPlugin.dir/rule

# Convenience name for target.
platformPlugin: landingPlat/CMakeFiles/platformPlugin.dir/rule

.PHONY : platformPlugin

# fast build rule for target.
platformPlugin/fast:
	cd /home/andreanistico/Work/GazeboPlugins && $(MAKE) -f landingPlat/CMakeFiles/platformPlugin.dir/build.make landingPlat/CMakeFiles/platformPlugin.dir/build
.PHONY : platformPlugin/fast

src/WaveGen.o: src/WaveGen.cpp.o

.PHONY : src/WaveGen.o

# target to build an object file
src/WaveGen.cpp.o:
	cd /home/andreanistico/Work/GazeboPlugins && $(MAKE) -f landingPlat/CMakeFiles/platformPlugin.dir/build.make landingPlat/CMakeFiles/platformPlugin.dir/src/WaveGen.cpp.o
.PHONY : src/WaveGen.cpp.o

src/WaveGen.i: src/WaveGen.cpp.i

.PHONY : src/WaveGen.i

# target to preprocess a source file
src/WaveGen.cpp.i:
	cd /home/andreanistico/Work/GazeboPlugins && $(MAKE) -f landingPlat/CMakeFiles/platformPlugin.dir/build.make landingPlat/CMakeFiles/platformPlugin.dir/src/WaveGen.cpp.i
.PHONY : src/WaveGen.cpp.i

src/WaveGen.s: src/WaveGen.cpp.s

.PHONY : src/WaveGen.s

# target to generate assembly for a file
src/WaveGen.cpp.s:
	cd /home/andreanistico/Work/GazeboPlugins && $(MAKE) -f landingPlat/CMakeFiles/platformPlugin.dir/build.make landingPlat/CMakeFiles/platformPlugin.dir/src/WaveGen.cpp.s
.PHONY : src/WaveGen.cpp.s

src/plugin.o: src/plugin.cpp.o

.PHONY : src/plugin.o

# target to build an object file
src/plugin.cpp.o:
	cd /home/andreanistico/Work/GazeboPlugins && $(MAKE) -f landingPlat/CMakeFiles/platformPlugin.dir/build.make landingPlat/CMakeFiles/platformPlugin.dir/src/plugin.cpp.o
.PHONY : src/plugin.cpp.o

src/plugin.i: src/plugin.cpp.i

.PHONY : src/plugin.i

# target to preprocess a source file
src/plugin.cpp.i:
	cd /home/andreanistico/Work/GazeboPlugins && $(MAKE) -f landingPlat/CMakeFiles/platformPlugin.dir/build.make landingPlat/CMakeFiles/platformPlugin.dir/src/plugin.cpp.i
.PHONY : src/plugin.cpp.i

src/plugin.s: src/plugin.cpp.s

.PHONY : src/plugin.s

# target to generate assembly for a file
src/plugin.cpp.s:
	cd /home/andreanistico/Work/GazeboPlugins && $(MAKE) -f landingPlat/CMakeFiles/platformPlugin.dir/build.make landingPlat/CMakeFiles/platformPlugin.dir/src/plugin.cpp.s
.PHONY : src/plugin.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... rebuild_cache"
	@echo "... platformPlugin"
	@echo "... src/WaveGen.o"
	@echo "... src/WaveGen.i"
	@echo "... src/WaveGen.s"
	@echo "... src/plugin.o"
	@echo "... src/plugin.i"
	@echo "... src/plugin.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	cd /home/andreanistico/Work/GazeboPlugins && $(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system
