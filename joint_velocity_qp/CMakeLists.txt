cmake_minimum_required(VERSION 3.18)
project(joint_velocity_qp VERSION 1.0.0 LANGUAGES C CXX)

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(osqp 1.0 CONFIG QUIET)
option(JOINT_VELOCITY_QP_FETCH_OSQP "Download OSQP 1.0.0 if not installed" ON)

if(NOT osqp_FOUND)
  if(NOT JOINT_VELOCITY_QP_FETCH_OSQP)
    message(FATAL_ERROR "OSQP 1.x not found. Set osqp_DIR or enable JOINT_VELOCITY_QP_FETCH_OSQP.")
  endif()
  include(FetchContent)
  set(OSQP_VERSION "1.0.0" CACHE STRING "OSQP version")
  set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
  set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL "" FORCE)
  set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
  set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
  set(OSQP_CODEGEN OFF CACHE BOOL "" FORCE)
  set(OSQP_ENABLE_DERIVATIVES OFF CACHE BOOL "" FORCE)
  set(OSQP_ENABLE_PRINTING OFF CACHE BOOL "" FORCE)
  set(OSQP_USE_FLOAT OFF CACHE BOOL "" FORCE)
  # Declare QDLDL first to override OSQP's Git download with a pinned archive.
  FetchContent_Declare(qdldl
    URL https://codeload.github.com/osqp/qdldl/tar.gz/refs/tags/v0.1.8
    URL_HASH SHA256=ecf113fd6ad8714f16289eb4d5f4d8b27842b6775b978c39def5913f983f6daa)
  FetchContent_Declare(osqp
    URL https://codeload.github.com/osqp/osqp/tar.gz/refs/tags/v1.0.0
    URL_HASH SHA256=dd6a1c2e7e921485697d5e7cdeeb043c712526c395b3700601f51d472a7d8e48)
  FetchContent_MakeAvailable(osqp)
endif()

if(TARGET osqp::osqpstatic)
  set(qp_osqp_target osqp::osqpstatic)
elseif(TARGET osqp::osqp)
  set(qp_osqp_target osqp::osqp)
elseif(TARGET osqpstatic)
  set(qp_osqp_target osqpstatic)
else()
  message(FATAL_ERROR "No supported OSQP target found")
endif()

add_library(joint_velocity_qp src/solver.cpp)
add_library(joint_velocity_qp::joint_velocity_qp ALIAS joint_velocity_qp)
target_compile_features(joint_velocity_qp PUBLIC cxx_std_17)
target_include_directories(joint_velocity_qp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(joint_velocity_qp PUBLIC Eigen3::Eigen PRIVATE ${qp_osqp_target})
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(joint_velocity_qp PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(joint_velocity_qp_cli src/main.cpp)
target_link_libraries(joint_velocity_qp_cli PRIVATE joint_velocity_qp)

include(CTest)
if(BUILD_TESTING)
  add_executable(joint_velocity_qp_tests tests/solver_test.cpp)
  target_link_libraries(joint_velocity_qp_tests PRIVATE joint_velocity_qp)
  add_test(NAME joint_velocity_qp_solver COMMAND joint_velocity_qp_tests)
  add_test(NAME joint_velocity_qp_demo COMMAND joint_velocity_qp_cli --demo)
  add_test(NAME joint_velocity_qp_cli_input
    COMMAND ${CMAKE_COMMAND}
      -DPROGRAM=$<TARGET_FILE:joint_velocity_qp_cli>
      -DINPUT=${CMAKE_CURRENT_SOURCE_DIR}/examples/input.txt
      -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/cli_test.cmake)
endif()
