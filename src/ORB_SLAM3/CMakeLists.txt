cmake_minimum_required(VERSION 3.0)  # Minimum required version
project(ORB_SLAM3)

# ------------------ Set Build Type ------------------
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

# ------------------ Compiler Optimization Flags ------------------
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -Wno-deprecated-declarations -Wno-error")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# ------------------ Ensure C++11 Support ------------------
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} does not support C++11. Please use a compatible compiler.")
endif()

# ------------------ Set CMake Module Path ------------------
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# ------------------ Explicitly Define OpenCV & Pangolin Paths ------------------
set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")
set(Pangolin_DIR "/usr/local/lib/cmake/Pangolin")

# ------------------ Find OpenCV (Version 4.4 Installed) ------------------
find_package(OpenCV 4.4 REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 4.4 not found.")
endif()
MESSAGE("OPENCV VERSION: ${OpenCV_VERSION}")

# ------------------ Find Required Dependencies ------------------
find_package(Eigen3 3.1.0 REQUIRED)  # Eigen3 for matrix operations
find_package(Pangolin REQUIRED)       # Pangolin for visualization
find_package(realsense2)              # Intel RealSense SDK

# ------------------ Include Directories ------------------
include_directories(
    ${PROJECT_SOURCE_DIR}                       # Project Root
    ${PROJECT_SOURCE_DIR}/include               # ORB-SLAM3 Header Files
    ${PROJECT_SOURCE_DIR}/include/CameraModels  # Camera Models
    ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus     # Sophus for Lie groups
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2      # ✅ Add main DBoW2 folder
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DBoW2 # ✅ Add nested DBoW2/DBoW2 folder
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o        # ✅ Add g2o (might be needed)
    ${EIGEN3_INCLUDE_DIR}                       # Eigen3
    ${Pangolin_INCLUDE_DIRS}                    # Pangolin
    ${OpenCV_INCLUDE_DIRS}                      # OpenCV Headers
)

# ------------------ Set Library Output Directory ------------------
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# ------------------ Add ORB-SLAM3 Shared Library ------------------
add_library(${PROJECT_NAME} SHARED
    src/System.cc
    src/Tracking.cc
    src/LocalMapping.cc
    src/LoopClosing.cc
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/FrameDrawer.cc
    src/Converter.cc
    src/MapPoint.cc
    src/KeyFrame.cc
    src/Atlas.cc
    src/Map.cc
    src/MapDrawer.cc
    src/Optimizer.cc
    src/Frame.cc
    src/KeyFrameDatabase.cc
    src/Sim3Solver.cc
    src/Viewer.cc
    src/ImuTypes.cc
    src/G2oTypes.cc
    src/CameraModels/Pinhole.cpp
    src/CameraModels/KannalaBrandt8.cpp
    src/OptimizableTypes.cpp
    src/MLPnPsolver.cpp
    src/GeometricTools.cc
    src/TwoViewReconstruction.cc
    src/Config.cc
    src/Settings.cc
    include/System.h
    include/Tracking.h
    include/LocalMapping.h
    include/LoopClosing.h
    include/ORBextractor.h
    include/ORBmatcher.h
    include/FrameDrawer.h
    include/Converter.h
    include/MapPoint.h
    include/KeyFrame.h
    include/Atlas.h
    include/Map.h
    include/MapDrawer.h
    include/Optimizer.h
    include/Frame.h
    include/KeyFrameDatabase.h
    include/Sim3Solver.h
    include/Viewer.h
    include/ImuTypes.h
    include/G2oTypes.h
    include/CameraModels/GeometricCamera.h
    include/CameraModels/Pinhole.h
    include/CameraModels/KannalaBrandt8.h
    include/OptimizableTypes.h
    include/MLPnPsolver.h
    include/GeometricTools.h
    include/TwoViewReconstruction.h
    include/Config.h
    include/Settings.h
)

# ------------------ Add Third-Party Libraries ------------------
add_subdirectory(Thirdparty/g2o)

# ------------------ Link Libraries ------------------
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}                   # OpenCV
    ${EIGEN3_LIBS}                   # Eigen3
    ${Pangolin_LIBRARIES}             # Pangolin
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
    -lboost_serialization
    -lcrypto
)

# ------------------ RealSense SDK Integration ------------------
if(realsense2_FOUND)
    include_directories(${PROJECT_NAME} ${realsense_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
endif()

# ------------------ Build ORB-SLAM3 Examples ------------------

# 🔹 RGB-D Examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)
add_executable(rgbd_tum Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(rgbd_realsense_D435i Examples/RGB-D/rgbd_realsense_D435i.cc)
    target_link_libraries(rgbd_realsense_D435i ${PROJECT_NAME})
endif()

# 🔹 Monocular Examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)
add_executable(mono_tum Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(mono_realsense_D435i Examples/Monocular/mono_realsense_D435i.cc)
    target_link_libraries(mono_realsense_D435i ${PROJECT_NAME})
endif()

# 🔹 Stereo Examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)
add_executable(stereo_kitti Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

if(realsense2_FOUND)
    add_executable(stereo_realsense_D435i Examples/Stereo/stereo_realsense_D435i.cc)
    target_link_libraries(stereo_realsense_D435i ${PROJECT_NAME})
endif()
