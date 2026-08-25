cmake_minimum_required(VERSION 3.10)

project(rl_controller)

set(CMAKE_BUILD_TYPE Release)
add_definitions(-w) # warning ignore
add_compile_options(-fPIC)

# ========== 控制选项 ==========
set(BUILD_PLATFORM "x86" CACHE STRING "select build cpu type")
option(BUILD_SIM OFF)
option(USE_RAISIM OFF)
set(USE_PYBULLET ON)
set(USE_RAISIM OFF)
option(SEND_REMOTE OFF)
# option(USE_ONNX "Use ONNXRuntime" ON)
option(USE_MJCPP "Use Mujoco C++" OFF)



# if (BUILD_PLATFORM STREQUAL arm)
#   message("this is arm platform")
#   set(BUILD_SIM OFF)
#   set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
#   set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
# else()
#   message("this is x86 platform")
# endif()
set(CMAKE_CXX_STANDARD 17)

if (SEND_REMOTE)
  set(BUILD_SIM OFF)
endif()

# ========== 显示参数 ==========
message("BUILD_PLATFORM: ${BUILD_PLATFORM}")
message("BUILD_SIM:      ${BUILD_SIM}")
message("SEND_REMOTE:    ${SEND_REMOTE}")
# message("USE_ONNX:       ${USE_ONNX}")
message("USE_MJCPP:      ${USE_MJCPP}")

# 路径设置
get_filename_component(WORKSPACE_DIR ./ ABSOLUTE)
set(THIRD_PARTY ${WORKSPACE_DIR}/third_party)

# torch setting
# set(Torch_DIR "${THIRD_PARTY}/libtorch/${BUILD_PLATFORM}/share/cmake/Torch")
# find_package(Torch REQUIRED)
# message("Torch_DIR:  ${Torch_DIR}")

# ONNX
set(ONNX_RUNTIME_ROOT "${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}")
message("Using ONNXRuntime from: ${ONNX_RUNTIME_ROOT}")


# ONNX setting
# if (USE_ONNX)
#   add_definitions(-DUSE_ONNX)

#   include_directories(${ONNX_RUNTIME_ROOT}/include)
#   link_directories(${ONNX_RUNTIME_ROOT}/lib)
# endif()
include_directories(${ONNX_RUNTIME_ROOT}/include)
link_directories(${ONNX_RUNTIME_ROOT}/lib)


# MuJoCo
set(MUJOCO_ROOT "${THIRD_PARTY}/mujoco/${BUILD_PLATFORM}")
message("USE_MJCPP enabled, loading MuJoCo headers from ${MUJOCO_ROOT}")
if (USE_MJCPP)
  add_definitions(-DUSE_MJCPP)
  include_directories(${MUJOCO_ROOT}/include)
  link_directories(${MUJOCO_ROOT}/lib)
endif()





# ========== Include 头文件路径 ==========
include_directories(
  /usr/include/eigen3
  /usr/include
  ${THIRD_PARTY}/Lite3_MotionSDK/lib
  ${THIRD_PARTY}/eigen/
  ${THIRD_PARTY}/gamepad/include
  ${THIRD_PARTY}/Lite3_MotionSDK/include/common
  ${THIRD_PARTY}/Lite3_MotionSDK/include
  types
  utils
  interface/robot
  interface/user_command
  state_machine
  # ${TORCH_INCLUDE_DIRS}
  ${ONNX_RUNTIME_ROOT}/include
  ${MUJOCO_ROOT}/include
)

# ========== 添加 interface 子模块 ==========
add_subdirectory(interface)

# ========== CMPC Controller (MIT Convex MPC) ==========
# cmpc_ctrl/src must be set BEFORE add_subdirectory so qpOASES inherits it.
# qpOASES headers use <qpOASES/include/qpOASES/...> paths relative to cmpc_ctrl/src.
# Safe to add globally: no types.h conflict here (JCQP/types.h is deeper).
include_directories(
  cmpc_ctrl/src
  cmpc_ctrl/src/Controllers
  cmpc_ctrl/src/StateEstimator
  cmpc_ctrl/src/StateEstimator/include
)

add_subdirectory(cmpc_ctrl/src/JCQP)
add_subdirectory(cmpc_ctrl/src/osqp)
add_subdirectory(cmpc_ctrl/src/qpOASES)

set(CMPC_SRCS
  cmpc_ctrl/src/GaitCtrller.cpp
  cmpc_ctrl/src/calculateTool.cpp
  cmpc_ctrl/src/StateEstimator/ContactEstimator.cpp
  cmpc_ctrl/src/Controllers/DesiredStateCommand.cpp
  cmpc_ctrl/src/Controllers/FootSwingTrajectory.cpp
  cmpc_ctrl/src/Controllers/LegController.cpp
  cmpc_ctrl/src/Controllers/GeneralizedMomentumObserver.cpp
  cmpc_ctrl/src/Controllers/GroundReactionForceEstimator.cpp
  cmpc_ctrl/src/StateEstimator/OrientationEstimator.cpp
  cmpc_ctrl/src/StateEstimator/PositionVelocityEstimator.cpp
  cmpc_ctrl/src/Controllers/SafetyChecker.cpp
  cmpc_ctrl/src/Dynamics/FloatingBaseModel.cpp
  cmpc_ctrl/src/Dynamics/Quadruped.cpp
  cmpc_ctrl/src/Pinocchio_Dynamics/Lite3DynamicModel.cpp
  cmpc_ctrl/src/Pinocchio_Dynamics/Lite3StateMapper.cpp
  cmpc_ctrl/src/MPC_Ctrl/ConvexMPCLocomotion.cpp
  cmpc_ctrl/src/MPC_Ctrl/Gait.cpp
  cmpc_ctrl/src/MPC_Ctrl/SparseCMPC.cpp
  cmpc_ctrl/src/MPC_Ctrl/SparseCMPC_Math.cpp
  cmpc_ctrl/src/MPC_Ctrl/OsqpTriples.cpp
  cmpc_ctrl/src/MPC_Ctrl/convexMPC_interface.cpp
  cmpc_ctrl/src/MPC_Ctrl/SolverMPC.cpp
  cmpc_ctrl/src/MPC_Ctrl/RobotState.cpp
)

# ========== 仿真器宏定义 ==========
if (BUILD_SIM)
message("USE_PYBULLET:    ${USE_PYBULLET}")

  if(USE_RAISIM)
    add_definitions(-DUSE_RAISIM)
    set(raisim_DIR "your path to raisim")
    find_package(raisim CONFIG REQUIRED)
  elseif(USE_PYBULLET)
    add_definitions(-DUSE_PYBULLET)
  elseif(USE_MJCPP)
    set(MUJOCO_ROOT "${THIRD_PARTY}/mujoco/${BUILD_PLATFORM}")
    message("USE_MJCPP enabled, loading MuJoCo headers from ${MUJOCO_ROOT}")
    add_definitions(-DUSE_MJCPP)
    include_directories(${MUJOCO_ROOT}/include)
    link_directories(${MUJOCO_ROOT}/lib)
  else()
    message(FATAL_ERROR "No simulation option selected. Please select one of USE_RAISIM, USE_PYBULLET, or USE_MJCPP.")
  endif()
endif()

set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/../third_party/Lite3_MotionSDK/lib:/home/ysc/workspaces/tuanpm/lite3_mpc/third_party/Lite3_MotionSDK/lib:/home/ysc/cmpc_deploy/lib:/usr/local/lib")


# ========== 快速链接器 (可选) ==========
# Install mold with: sudo apt install mold
# Then re-run cmake.  Falls back to system default if not found.
option(USE_MOLD_LINKER "Use mold linker for faster linking" OFF)
if(USE_MOLD_LINKER)
    find_program(MOLD_EXECUTABLE mold)
    if(MOLD_EXECUTABLE)
        add_link_options(-fuse-ld=mold)
        message("Using mold linker: ${MOLD_EXECUTABLE}")
    else()
        message(WARNING "USE_MOLD_LINKER=ON but mold not found; using default linker")
    endif()
endif()

# ========== 收集源文件 ==========
file(GLOB_RECURSE STATE_MACHINE_SRC "state_machine/*.c*")

add_executable(cmpc_deploy main.cpp ${STATE_MACHINE_SRC} ${CMPC_SRCS})

target_include_directories(cmpc_deploy PRIVATE
  ${THIRD_PARTY}
  ${THIRD_PARTY}/boost
  cmpc_ctrl/src/JCQP
  cmpc_ctrl/src/JCQP/amd/include
  cmpc_ctrl/src/osqp/include
  cmpc_ctrl/src/qpOASES/include
  cmpc_ctrl/src/Utilities
  cmpc_ctrl/src/Controllers
  cmpc_ctrl/src/StateEstimator
  cmpc_ctrl/src/StateEstimator/include
  cmpc_ctrl/src/Pinocchio_Dynamics/include
)

# ========== 链接库 ==========
target_link_libraries(cmpc_deploy
    interface
    JCQP
    qpOASES
    osqpstatic
)

# SDK动态链接库
if (BUILD_SIM)
  link_directories(${THIRD_PARTY}/Lite3_MotionSDK/lib)
  target_link_libraries(cmpc_deploy
      ${PROJECT_SOURCE_DIR}/third_party/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_x86_64.so
  )
else()
  target_link_libraries(cmpc_deploy
      ${PROJECT_SOURCE_DIR}/third_party/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_aarch64.so
  )
endif()

if(BUILD_SIM)
    target_link_libraries(cmpc_deploy dw)
endif()

# System libraries dat o cuoi de giai quyet toan bo symbol dlopen, math, thread
target_link_libraries(cmpc_deploy
    pthread
    rt
    dl
    m
    stdc++
)

if (BUILD_SIM)
    target_compile_definitions(cmpc_deploy PRIVATE BUILD_SIMULATION)
endif()

# SDK动态链接库
# link_directories(${THIRD_PARTY}/Lite3_MotionSDK/lib)
# target_link_libraries(cmpc_deploy
#     ${PROJECT_SOURCE_DIR}/third_party/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_x86_64.so
#     pthread
#     rt
# )

# for real
# target_link_libraries(cmpc_deploy
#     ${PROJECT_SOURCE_DIR}/third_party/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_aarch64.so
#     pthread
#     rt
# )




# Raisim
if (BUILD_SIM)
  if(USE_RAISIM)
    message("build simulation raisim")
    target_link_libraries(cmpc_deploy raisim::raisim)
  endif()
endif()

# ========== Test Executables (Pinocchio & Dynamics Verification) ==========
option(BUILD_TESTS "Build verification and smoke tests" ON)
if(BUILD_TESTS)
  enable_testing()
  add_executable(pinocchio_smoke tests/pinocchio_smoke.cpp)
  target_include_directories(pinocchio_smoke PRIVATE
    ${THIRD_PARTY}
    ${THIRD_PARTY}/boost
  )
  target_link_libraries(pinocchio_smoke -lpthread -lm -lrt -ldl -lstdc++)
  add_test(NAME pinocchio_smoke COMMAND pinocchio_smoke)

  add_executable(lite3_pinocchio_model_smoke
    tests/lite3_pinocchio_model_smoke.cpp
    cmpc_ctrl/src/Pinocchio_Dynamics/Lite3DynamicModel.cpp
  )
  target_include_directories(lite3_pinocchio_model_smoke PRIVATE
    ${THIRD_PARTY}
    ${THIRD_PARTY}/boost
    cmpc_ctrl/src/Pinocchio_Dynamics/include
  )
  target_link_libraries(lite3_pinocchio_model_smoke -lpthread -lm -lrt -ldl -lstdc++)
  add_test(NAME lite3_pinocchio_model_smoke COMMAND lite3_pinocchio_model_smoke)

  add_executable(lite3_gmo_smoke
    tests/lite3_gmo_smoke.cpp
    cmpc_ctrl/src/Pinocchio_Dynamics/Lite3DynamicModel.cpp
    cmpc_ctrl/src/Pinocchio_Dynamics/Lite3StateMapper.cpp
    cmpc_ctrl/src/Controllers/GeneralizedMomentumObserver.cpp
    cmpc_ctrl/src/Controllers/GroundReactionForceEstimator.cpp
    cmpc_ctrl/src/Controllers/LegController.cpp
  )
  target_include_directories(lite3_gmo_smoke PRIVATE
    ${THIRD_PARTY}
    ${THIRD_PARTY}/boost
    cmpc_ctrl/src
    cmpc_ctrl/src/Controllers
    cmpc_ctrl/src/StateEstimator
    cmpc_ctrl/src/StateEstimator/include
    cmpc_ctrl/src/Pinocchio_Dynamics/include
  )
  target_link_libraries(lite3_gmo_smoke -lpthread -lm -lrt -ldl -lstdc++)
  add_test(NAME lite3_gmo_smoke COMMAND lite3_gmo_smoke)
endif()

if (SEND_REMOTE)
  add_custom_target(push_to_robot ALL DEPENDS cmpc_deploy)
  add_custom_command(
    TARGET push_to_robot POST_BUILD
    COMMAND ./scripts/sftp_to_remote.sh
    WORKING_DIRECTORY ..
    COMMENT "run sftp_to_remote.sh"
  )
endif()
