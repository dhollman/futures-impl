
cmake_minimum_required(VERSION 3.10)
project(StdFutures
  VERSION 1.0.0
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)

################################################################################

option(FUTURES_ENABLE_TESTING "Enable tests." Off)
option(FUTURES_ENABLE_EXAMPLES "Build examples." Off)

################################################################################

add_library(futures INTERFACE)
add_library(std::futures ALIAS futures)

target_include_directories(futures INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(futures INTERFACE cxx_std_17)

################################################################################

find_package(StdExecutors REQUIRED)
message(STATUS "Found P0443 executors: ${StdExecutors_DIR}")
target_link_libraries(futures INTERFACE std::executors)

################################################################################

install(TARGETS futures EXPORT futuresTargets
  INCLUDES DESTINATION include
)

install(EXPORT futuresTargets
    FILE futuresTargets.cmake
    NAMESPACE std::
    DESTINATION cmake
)

export(TARGETS futures
    NAMESPACE std::
    FILE futuresTargets.cmake
)

install(DIRECTORY include/experimental DESTINATION include)
install(DIRECTORY include/experimental DESTINATION include/__p1054)

include(CMakePackageConfigHelpers)
configure_package_config_file(cmake/StdFuturesConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/StdFuturesConfig.cmake
  INSTALL_DESTINATION cmake
)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/StdFuturesConfigVersion.cmake
  COMPATIBILITY SameMajorVersion
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/StdFuturesConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/StdFuturesConfigVersion.cmake
    DESTINATION cmake
)

################################################################################

if(FUTURES_ENABLE_TESTING)
  add_subdirectory(tests)
endif()

if(FUTURES_ENABLE_EXAMPLES)
  add_subdirectory(examples)
endif()

