add_library(aim_model STATIC model/armor.cpp)
target_link_libraries(aim_model PUBLIC dust::include Eigen3::Eigen opencv_core PRIVATE tools)

if(build_pipeline)
  add_library(aim_roi STATIC detection/dynamic_roi.cpp)
  target_link_libraries(aim_roi PUBLIC dust::include opencv_core)

  add_library(aim_geometry STATIC geometry/solver.cpp)
  target_link_libraries(aim_geometry
    PUBLIC aim_model PRIVATE tools yaml-cpp opencv_calib3d)

  add_library(aim_tracking STATIC tracking/target.cpp tracking/tracker.cpp)
  target_link_libraries(aim_tracking PUBLIC aim_geometry aim_roi tools PRIVATE yaml-cpp)

  add_subdirectory(planner/tinympc)
  add_library(aim_planner STATIC planner/planner.cpp planner/mpc_axis.cpp
    planner/axis_strategy.cpp planner/fire_evaluator.cpp planner/pitch_yaw_window.cpp)
  # mpc_axis.hpp includes TinyMPC types. No detection or hardware target is needed here.
  target_link_libraries(aim_planner PUBLIC aim_tracking tinympcstatic PRIVATE tools yaml-cpp)
  file(GLOB mpc_build_sources CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/planner/mpc_axis.*
    ${CMAKE_CURRENT_SOURCE_DIR}/planner/tinympc/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/planner/tinympc/*.hpp)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${mpc_build_sources})
  set(mpc_build_hashes "${CMAKE_CXX_COMPILER_ID};${CMAKE_CXX_COMPILER_VERSION};${CMAKE_BUILD_TYPE};${CMAKE_CXX_FLAGS};${CMAKE_CXX_FLAGS_RELEASE};${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
  foreach(source IN LISTS mpc_build_sources)
    file(SHA256 ${source} source_hash)
    string(APPEND mpc_build_hashes ${source_hash})
  endforeach()
  string(SHA256 mpc_build_id "${mpc_build_hashes}")
  target_compile_definitions(aim_planner PUBLIC DUST_MPC_BUILD_ID="${mpc_build_id}")

  # Target-reference replay also depends on prediction, selection, ballistics and differentiation.
  file(GLOB planner_build_sources CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/planner/planner.*
    ${CMAKE_CURRENT_SOURCE_DIR}/planner/axis_strategy.*
    ${CMAKE_CURRENT_SOURCE_DIR}/planner/fire_evaluator.*
    ${CMAKE_CURRENT_SOURCE_DIR}/planner/pitch_yaw_window.*
    ${CMAKE_CURRENT_SOURCE_DIR}/planner/reference.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tracking/target.*
    ${CMAKE_CURRENT_SOURCE_DIR}/model/armor.hpp
    ${PROJECT_SOURCE_DIR}/tools/math/trajectory.*
    ${PROJECT_SOURCE_DIR}/tools/math/math_tools.*
    ${PROJECT_SOURCE_DIR}/tools/runtime/yaml.*
    ${PROJECT_SOURCE_DIR}/tools/runtime/yaml_config.hpp)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${planner_build_sources})
  set(planner_build_hashes "${mpc_build_id}")
  foreach(source IN LISTS planner_build_sources)
    file(SHA256 ${source} source_hash)
    string(APPEND planner_build_hashes ${source_hash})
  endforeach()
  string(SHA256 planner_build_id "${planner_build_hashes}")
  target_compile_definitions(aim_planner PUBLIC DUST_PLANNER_BUILD_ID="${planner_build_id}")
endif()

# Classifier uses OpenCV DNN. OpenVINO headers are private to the network backend.
add_library(aim_traditional_detector STATIC detection/classifier.cpp detection/detector.cpp)
target_link_libraries(aim_traditional_detector
  PUBLIC aim_model opencv_dnn
  PRIVATE tools yaml-cpp fmt::fmt opencv_imgcodecs opencv_imgproc opencv_highgui)

if(build_inference)
  set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/" CACHE PATH
      "OpenVINO CMake package directory")
  find_package(OpenVINO REQUIRED COMPONENTS Runtime)
  if(ENABLE_TENSORRT)
    find_package(CUDAToolkit 12 REQUIRED)
    set(TensorRT_ROOT "$ENV{TRT_ROOT}" CACHE PATH "TensorRT installation root")
    find_path(TensorRT_INCLUDE_DIR NvInfer.h HINTS "${TensorRT_ROOT}/include")
    find_library(TensorRT_LIBRARY nvinfer HINTS "${TensorRT_ROOT}/lib" "${TensorRT_ROOT}/lib64")
    if(NOT TensorRT_INCLUDE_DIR OR NOT TensorRT_LIBRARY)
      message(FATAL_ERROR "TensorRT headers/library missing; set TensorRT_ROOT to a TensorRT 10 installation")
    endif()
    add_library(dust_tensorrt INTERFACE)
    target_include_directories(dust_tensorrt SYSTEM INTERFACE "${TensorRT_INCLUDE_DIR}")
    target_link_libraries(dust_tensorrt INTERFACE "${TensorRT_LIBRARY}" CUDA::cudart)
  endif()

  add_library(aim_detection STATIC
    detection/net_detector.cpp detection/yolo.cpp detection/yolov5.cpp detection/mt_detector.cpp)
  target_link_libraries(aim_detection
    PUBLIC aim_traditional_detector Threads::Threads
    PRIVATE tools openvino::runtime yaml-cpp fmt::fmt opencv_imgproc)
  if(ENABLE_TENSORRT)
    target_compile_definitions(aim_detection PRIVATE DUST_ENABLE_TENSORRT)
    target_link_libraries(aim_detection PRIVATE dust_tensorrt)
  endif()
endif()
