cmake_minimum_required(VERSION 3.24.2)
project(OpenCVExample)

# Set OpenCV directory paths
set(OpenCV_DIR "C:/brg/2_code/opencv/build")
set(OpenCV_INCLUDE_DIRS "C:/brg/2_code/opencv/build/include")
set(OpenCV_LIB_DIR "C:/brg/2_code/opencv/build/x64/vc16/lib")
set(OpenCV_BIN_DIR "C:/brg/2_code/opencv/build/x64/vc16/bin")

# Find OpenCV package
find_package(OpenCV REQUIRED PATHS ${OpenCV_DIR})

# Include directories from OpenCV
include_directories(${OpenCV_INCLUDE_DIRS})

# Create an executable
add_executable(OpenCVExample main.cpp)

# Link OpenCV libraries
target_link_libraries(OpenCVExample ${OpenCV_LIBS})

# Copy OpenCV DLLs to the output directory
add_custom_command(TARGET OpenCVExample POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${OpenCV_BIN_DIR} $<TARGET_FILE_DIR:OpenCVExample>)


# use
# mkdir build
# cmake -B .\build\
# cmake --build .\build\
# .\build\Debug\OpenCVExample.exe

# build in realease
# cmake --build .\build\ --config Release
# .\build\Release\OpenCVExample.exe

# if you change code rerun
# cmake --build .\build\

# cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build --config Release && .\build\Release\OpenCVExample.exe