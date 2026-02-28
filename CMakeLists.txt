cmake_minimum_required(VERSION 3.20)
project(opennest_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- External dependencies ---

# Clipper2 (vendored in-tree)
set(CLIPPER2_SOURCES
    src/clipper2/clipper.engine.cpp
    src/clipper2/clipper.offset.cpp
    src/clipper2/clipper.rectclip.cpp
)
add_library(Clipper2 STATIC ${CLIPPER2_SOURCES})
target_include_directories(Clipper2 PUBLIC ${CMAKE_SOURCE_DIR}/src)

# --- Source files ---
set(NEST_SOURCES
    src/Point.h
    src/NestConfig.h
    src/HelperTypes.h
    src/NFP.h
    src/GeometryUtil.h
    src/GeometryUtil.cpp
    src/Simplify.h
    src/D3.h
    src/ClipperUtil.h
    src/MinkowskiConvolution.h
    src/NfpWorker.h
    src/NfpWorker.cpp
    src/Random.h
    src/GeneticAlgorithm.h
    src/GeneticAlgorithm.cpp
    src/NestingEngine.h
    src/NestingEngine.cpp
    src/NestingContext.h
    src/NestingContext.cpp
)

# --- Nest static library ---
add_library(nest_lib STATIC ${NEST_SOURCES})
target_include_directories(nest_lib
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(nest_lib PUBLIC Clipper2)

# MSVC needs _USE_MATH_DEFINES for M_PI
if(MSVC)
    target_compile_definitions(nest_lib PUBLIC _USE_MATH_DEFINES)
    target_compile_options(nest_lib PRIVATE /W3 /wd4244 /wd4267 /wd4305)
else()
    target_compile_options(nest_lib PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

# --- Solver modules static library ---
set(SOLVER_SOURCES
    src/SolverConfig.h
    src/Compaction.h
    src/Compaction.cpp
    src/ContinuousRotation.h
    src/ContinuousRotation.cpp
    src/JostleHeuristic.h
    src/JostleHeuristic.cpp
    src/PairPreMatching.h
    src/PairPreMatching.cpp
    src/ExhaustiveSolver.h
    src/ExhaustiveSolver.cpp
    src/OverlapResolution.h
    src/OverlapResolution.cpp
    src/SolverOrchestrator.h
    src/SolverOrchestrator.cpp
)

add_library(solvers_lib STATIC ${SOLVER_SOURCES})
target_include_directories(solvers_lib PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(solvers_lib PUBLIC nest_lib)

if(MSVC)
    target_compile_options(solvers_lib PRIVATE /W3 /wd4244 /wd4267 /wd4305)
else()
    target_compile_options(solvers_lib PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

# --- Example applications ---

add_executable(opennest_app examples/main.cpp)
target_link_libraries(opennest_app PRIVATE nest_lib)
target_include_directories(opennest_app PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(advanced_example examples/advanced_example.cpp)
target_link_libraries(advanced_example PRIVATE nest_lib)
target_include_directories(advanced_example PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(curvy_example examples/curvy_example.cpp)
target_link_libraries(curvy_example PRIVATE nest_lib)
target_include_directories(curvy_example PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(solvers_app examples/main_solvers.cpp)
target_link_libraries(solvers_app PRIVATE solvers_lib)
target_include_directories(solvers_app PRIVATE ${CMAKE_SOURCE_DIR}/src)

# --- Verification tests ---

add_executable(verify_layer01 tests/verify_layer01.cpp)
target_link_libraries(verify_layer01 PRIVATE nest_lib)
target_include_directories(verify_layer01 PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(verify_layer2 tests/verify_layer2.cpp)
target_link_libraries(verify_layer2 PRIVATE nest_lib)
target_include_directories(verify_layer2 PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(verify_layer3 tests/verify_layer3.cpp)
target_link_libraries(verify_layer3 PRIVATE nest_lib)
target_include_directories(verify_layer3 PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(verify_layer4 tests/verify_layer4.cpp)
target_link_libraries(verify_layer4 PRIVATE nest_lib)
target_include_directories(verify_layer4 PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(verify_layer5 tests/verify_layer5.cpp)
target_link_libraries(verify_layer5 PRIVATE nest_lib)
target_include_directories(verify_layer5 PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(verify_layer6 tests/verify_layer6.cpp)
target_link_libraries(verify_layer6 PRIVATE nest_lib)
target_include_directories(verify_layer6 PRIVATE ${CMAKE_SOURCE_DIR}/src)

add_executable(verify_nfp_groundtruth tests/verify_nfp_groundtruth.cpp)
target_link_libraries(verify_nfp_groundtruth PRIVATE nest_lib)
target_include_directories(verify_nfp_groundtruth PRIVATE ${CMAKE_SOURCE_DIR}/src)

# Solver verification tests
add_executable(verify_solvers tests/verify_solvers.cpp)
target_link_libraries(verify_solvers PRIVATE solvers_lib)
target_include_directories(verify_solvers PRIVATE ${CMAKE_SOURCE_DIR}/src)
