cmake_minimum_required(VERSION 3.5)
Project(CoppeliaSim_remote)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin/Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin/Release)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

aux_source_directory(src SRC_MAIN)
aux_source_directory(src/controller SRC_CONTROLLER)
aux_source_directory(src/coppeliasim SRC_COPPELIASIM)
aux_source_directory(CoppeliaSimApi_V4.3.0/remote_api SRC_COPPELIASIM)

add_definitions(-DMAX_EXT_API_CONNECTIONS=255 -DNON_MATLAB_PARSING -DDO_NOT_USE_SHARED_MEMORY -D_CRT_SECURE_NO_WARNINGS
 -D_USE_MATH_DEFINES)
add_executable(test
  ${SRC_MAIN}
  ${SRC_CONTROLLER}
  ${SRC_COPPELIASIM}
)
target_include_directories(test PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/coppeliasim
  ${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaSimApi_V4.3.0/remote_api
  ${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaSimApi_V4.3.0/include
)

