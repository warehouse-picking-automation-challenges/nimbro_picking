
cmake_minimum_required(VERSION 2.8)
project(densecap_accel)

find_package(OpenCV REQUIRED)

find_package(Torch)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")

add_executable(nms_bench
  nms_bench.cpp
  nms.cpp
)

add_torch_package(densecap_accel
  "nms.cpp;accel.cpp"
  "init.lua"
  "Densecap acceleration"
)
target_link_libraries(densecap_accel
  luaT TH
)
