##############################################################################
# CMake-based build system for KinectFusionExplorer-D2D
# Author: Seung-Tak Noh (seungtak.noh@gmail.com)
##############################################################################
add_definitions(-DUNICODE -D_UNICODE) ## KinectFusion only supports Unicode

file(GLOB 
  headers
  "*.h" "*.rc"
)

file(GLOB
  sources
  "*.cpp" "*.cc"
)

add_executable (KinectFusionExplorer WIN32 ${headers} ${sources})

target_include_directories(KinectFusionExplorer PUBLIC
  cvgl ${CMAKE_SOURCE_DIR}/external/pugixml/src
)

target_link_libraries(KinectFusionExplorer cvgl Kinect2::Core Kinect2::Fusion protobuf::libprotobuf RSSDK::Core)

add_custom_command(TARGET KinectFusionExplorer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${KINECT2_ROOT}/Redist/Fusion/x64/Kinect20.Fusion.dll ${CMAKE_CURRENT_BINARY_DIR}
)

copy_3rdparty_dlls(KinectFusionExplorer)
