##############################################################################
# CMake-based build system for KinectFusionExplorer-D2D
# Author: Seung-Tak Noh (seungtak.noh@gmail.com)
##############################################################################
cmake_minimum_required (VERSION 3.1)
project ("KinectFusionExplorer")

add_definitions(-DUNICODE -D_UNICODE) ## KinectFusion only supports Unicode

set(KINECTSDK20_DIR "C:/Program Files/Microsoft SDKs/Kinect/v2.0_1409")

file(GLOB 
  headers
  "*.h" "*.rc"
)

file(GLOB
  sources
  "*.cpp"
)

add_executable (KinectFusionExplorer WIN32 ${headers} ${sources})

target_include_directories(KinectFusionExplorer PUBLIC
  ${KINECTSDK20_DIR}/inc
)
target_link_libraries(KinectFusionExplorer
  ${KINECTSDK20_DIR}/lib/x64/Kinect20.lib
  ${KINECTSDK20_DIR}/lib/x64/Kinect20.Fusion.lib
)

add_custom_command(TARGET KinectFusionExplorer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${KINECTSDK20_DIR}/Redist/Fusion/x64/Kinect20.Fusion.dll ${CMAKE_CURRENT_BINARY_DIR}
)
