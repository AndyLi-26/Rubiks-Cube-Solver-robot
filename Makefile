# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_SOURCE_DIR = /home/pi/Desktop/raspi_rubiks

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Desktop/raspi_rubiks

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/pi/Desktop/raspi_rubiks/CMakeFiles /home/pi/Desktop/raspi_rubiks/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/pi/Desktop/raspi_rubiks/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named main

# Build rule for target.
main: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 main
.PHONY : main

# fast build rule for target.
main/fast:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/build
.PHONY : main/fast

main.o: main.cpp.o

.PHONY : main.o

# target to build an object file
main.cpp.o:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/main.cpp.o
.PHONY : main.cpp.o

main.i: main.cpp.i

.PHONY : main.i

# target to preprocess a source file
main.cpp.i:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/main.cpp.i
.PHONY : main.cpp.i

main.s: main.cpp.s

.PHONY : main.s

# target to generate assembly for a file
main.cpp.s:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/main.cpp.s
.PHONY : main.cpp.s

solving_algorithm/Combinatorics.o: solving_algorithm/Combinatorics.cpp.o

.PHONY : solving_algorithm/Combinatorics.o

# target to build an object file
solving_algorithm/Combinatorics.cpp.o:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/Combinatorics.cpp.o
.PHONY : solving_algorithm/Combinatorics.cpp.o

solving_algorithm/Combinatorics.i: solving_algorithm/Combinatorics.cpp.i

.PHONY : solving_algorithm/Combinatorics.i

# target to preprocess a source file
solving_algorithm/Combinatorics.cpp.i:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/Combinatorics.cpp.i
.PHONY : solving_algorithm/Combinatorics.cpp.i

solving_algorithm/Combinatorics.s: solving_algorithm/Combinatorics.cpp.s

.PHONY : solving_algorithm/Combinatorics.s

# target to generate assembly for a file
solving_algorithm/Combinatorics.cpp.s:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/Combinatorics.cpp.s
.PHONY : solving_algorithm/Combinatorics.cpp.s

solving_algorithm/Cube.o: solving_algorithm/Cube.cpp.o

.PHONY : solving_algorithm/Cube.o

# target to build an object file
solving_algorithm/Cube.cpp.o:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/Cube.cpp.o
.PHONY : solving_algorithm/Cube.cpp.o

solving_algorithm/Cube.i: solving_algorithm/Cube.cpp.i

.PHONY : solving_algorithm/Cube.i

# target to preprocess a source file
solving_algorithm/Cube.cpp.i:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/Cube.cpp.i
.PHONY : solving_algorithm/Cube.cpp.i

solving_algorithm/Cube.s: solving_algorithm/Cube.cpp.s

.PHONY : solving_algorithm/Cube.s

# target to generate assembly for a file
solving_algorithm/Cube.cpp.s:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/Cube.cpp.s
.PHONY : solving_algorithm/Cube.cpp.s

solving_algorithm/CubeParser.o: solving_algorithm/CubeParser.cpp.o

.PHONY : solving_algorithm/CubeParser.o

# target to build an object file
solving_algorithm/CubeParser.cpp.o:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/CubeParser.cpp.o
.PHONY : solving_algorithm/CubeParser.cpp.o

solving_algorithm/CubeParser.i: solving_algorithm/CubeParser.cpp.i

.PHONY : solving_algorithm/CubeParser.i

# target to preprocess a source file
solving_algorithm/CubeParser.cpp.i:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/CubeParser.cpp.i
.PHONY : solving_algorithm/CubeParser.cpp.i

solving_algorithm/CubeParser.s: solving_algorithm/CubeParser.cpp.s

.PHONY : solving_algorithm/CubeParser.s

# target to generate assembly for a file
solving_algorithm/CubeParser.cpp.s:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/CubeParser.cpp.s
.PHONY : solving_algorithm/CubeParser.cpp.s

solving_algorithm/FaceletCube.o: solving_algorithm/FaceletCube.cpp.o

.PHONY : solving_algorithm/FaceletCube.o

# target to build an object file
solving_algorithm/FaceletCube.cpp.o:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/FaceletCube.cpp.o
.PHONY : solving_algorithm/FaceletCube.cpp.o

solving_algorithm/FaceletCube.i: solving_algorithm/FaceletCube.cpp.i

.PHONY : solving_algorithm/FaceletCube.i

# target to preprocess a source file
solving_algorithm/FaceletCube.cpp.i:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/FaceletCube.cpp.i
.PHONY : solving_algorithm/FaceletCube.cpp.i

solving_algorithm/FaceletCube.s: solving_algorithm/FaceletCube.cpp.s

.PHONY : solving_algorithm/FaceletCube.s

# target to generate assembly for a file
solving_algorithm/FaceletCube.cpp.s:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/FaceletCube.cpp.s
.PHONY : solving_algorithm/FaceletCube.cpp.s

solving_algorithm/MoveTable.o: solving_algorithm/MoveTable.cpp.o

.PHONY : solving_algorithm/MoveTable.o

# target to build an object file
solving_algorithm/MoveTable.cpp.o:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/MoveTable.cpp.o
.PHONY : solving_algorithm/MoveTable.cpp.o

solving_algorithm/MoveTable.i: solving_algorithm/MoveTable.cpp.i

.PHONY : solving_algorithm/MoveTable.i

# target to preprocess a source file
solving_algorithm/MoveTable.cpp.i:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/MoveTable.cpp.i
.PHONY : solving_algorithm/MoveTable.cpp.i

solving_algorithm/MoveTable.s: solving_algorithm/MoveTable.cpp.s

.PHONY : solving_algorithm/MoveTable.s

# target to generate assembly for a file
solving_algorithm/MoveTable.cpp.s:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/MoveTable.cpp.s
.PHONY : solving_algorithm/MoveTable.cpp.s

solving_algorithm/PruningTable.o: solving_algorithm/PruningTable.cpp.o

.PHONY : solving_algorithm/PruningTable.o

# target to build an object file
solving_algorithm/PruningTable.cpp.o:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/PruningTable.cpp.o
.PHONY : solving_algorithm/PruningTable.cpp.o

solving_algorithm/PruningTable.i: solving_algorithm/PruningTable.cpp.i

.PHONY : solving_algorithm/PruningTable.i

# target to preprocess a source file
solving_algorithm/PruningTable.cpp.i:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/PruningTable.cpp.i
.PHONY : solving_algorithm/PruningTable.cpp.i

solving_algorithm/PruningTable.s: solving_algorithm/PruningTable.cpp.s

.PHONY : solving_algorithm/PruningTable.s

# target to generate assembly for a file
solving_algorithm/PruningTable.cpp.s:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/PruningTable.cpp.s
.PHONY : solving_algorithm/PruningTable.cpp.s

solving_algorithm/RubiksCube.o: solving_algorithm/RubiksCube.cpp.o

.PHONY : solving_algorithm/RubiksCube.o

# target to build an object file
solving_algorithm/RubiksCube.cpp.o:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/RubiksCube.cpp.o
.PHONY : solving_algorithm/RubiksCube.cpp.o

solving_algorithm/RubiksCube.i: solving_algorithm/RubiksCube.cpp.i

.PHONY : solving_algorithm/RubiksCube.i

# target to preprocess a source file
solving_algorithm/RubiksCube.cpp.i:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/RubiksCube.cpp.i
.PHONY : solving_algorithm/RubiksCube.cpp.i

solving_algorithm/RubiksCube.s: solving_algorithm/RubiksCube.cpp.s

.PHONY : solving_algorithm/RubiksCube.s

# target to generate assembly for a file
solving_algorithm/RubiksCube.cpp.s:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/RubiksCube.cpp.s
.PHONY : solving_algorithm/RubiksCube.cpp.s

solving_algorithm/Solver.o: solving_algorithm/Solver.cpp.o

.PHONY : solving_algorithm/Solver.o

# target to build an object file
solving_algorithm/Solver.cpp.o:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/Solver.cpp.o
.PHONY : solving_algorithm/Solver.cpp.o

solving_algorithm/Solver.i: solving_algorithm/Solver.cpp.i

.PHONY : solving_algorithm/Solver.i

# target to preprocess a source file
solving_algorithm/Solver.cpp.i:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/Solver.cpp.i
.PHONY : solving_algorithm/Solver.cpp.i

solving_algorithm/Solver.s: solving_algorithm/Solver.cpp.s

.PHONY : solving_algorithm/Solver.s

# target to generate assembly for a file
solving_algorithm/Solver.cpp.s:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/Solver.cpp.s
.PHONY : solving_algorithm/Solver.cpp.s

solving_algorithm/Vector.o: solving_algorithm/Vector.cpp.o

.PHONY : solving_algorithm/Vector.o

# target to build an object file
solving_algorithm/Vector.cpp.o:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/Vector.cpp.o
.PHONY : solving_algorithm/Vector.cpp.o

solving_algorithm/Vector.i: solving_algorithm/Vector.cpp.i

.PHONY : solving_algorithm/Vector.i

# target to preprocess a source file
solving_algorithm/Vector.cpp.i:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/Vector.cpp.i
.PHONY : solving_algorithm/Vector.cpp.i

solving_algorithm/Vector.s: solving_algorithm/Vector.cpp.s

.PHONY : solving_algorithm/Vector.s

# target to generate assembly for a file
solving_algorithm/Vector.cpp.s:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/solving_algorithm/Vector.cpp.s
.PHONY : solving_algorithm/Vector.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... rebuild_cache"
	@echo "... main"
	@echo "... edit_cache"
	@echo "... main.o"
	@echo "... main.i"
	@echo "... main.s"
	@echo "... solving_algorithm/Combinatorics.o"
	@echo "... solving_algorithm/Combinatorics.i"
	@echo "... solving_algorithm/Combinatorics.s"
	@echo "... solving_algorithm/Cube.o"
	@echo "... solving_algorithm/Cube.i"
	@echo "... solving_algorithm/Cube.s"
	@echo "... solving_algorithm/CubeParser.o"
	@echo "... solving_algorithm/CubeParser.i"
	@echo "... solving_algorithm/CubeParser.s"
	@echo "... solving_algorithm/FaceletCube.o"
	@echo "... solving_algorithm/FaceletCube.i"
	@echo "... solving_algorithm/FaceletCube.s"
	@echo "... solving_algorithm/MoveTable.o"
	@echo "... solving_algorithm/MoveTable.i"
	@echo "... solving_algorithm/MoveTable.s"
	@echo "... solving_algorithm/PruningTable.o"
	@echo "... solving_algorithm/PruningTable.i"
	@echo "... solving_algorithm/PruningTable.s"
	@echo "... solving_algorithm/RubiksCube.o"
	@echo "... solving_algorithm/RubiksCube.i"
	@echo "... solving_algorithm/RubiksCube.s"
	@echo "... solving_algorithm/Solver.o"
	@echo "... solving_algorithm/Solver.i"
	@echo "... solving_algorithm/Solver.s"
	@echo "... solving_algorithm/Vector.o"
	@echo "... solving_algorithm/Vector.i"
	@echo "... solving_algorithm/Vector.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system
