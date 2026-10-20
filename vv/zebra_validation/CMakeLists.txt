# vv 项目的顶层 CMake 配置文件。
#
# 主要负责：
# 1. 指定龙芯 LoongArch 交叉编译工具链；
# 2. 查找 OpenCV 依赖；
# 3. 递归收集 `src` 下源码文件；
# 4. 生成最终可执行程序 `vv`。

cmake_minimum_required(VERSION 3.5)

# ===== 工具链设置 =====
# 配置交叉编译工具链目录与 C / C++ 编译器。
set(TOOLCHAIN_DIR "/opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6" CACHE PATH "VV17 cross toolchain")
option(ZEBRA_NATIVE "Build natively on Linux using installed compiler" OFF)
if(NOT ZEBRA_NATIVE)
  set(CMAKE_C_COMPILER "${TOOLCHAIN_DIR}/bin/loongarch64-linux-gnu-gcc")
  set(CMAKE_CXX_COMPILER "${TOOLCHAIN_DIR}/bin/loongarch64-linux-gnu-g++")
  set(CMAKE_SYSTEM_NAME Linux)
  set(CMAKE_SYSTEM_PROCESSOR loongarch64)
endif()

# ===== 项目定义 =====
project(vv_zebra_test)

# ===== 关键修复：添加 pthread 标志 =====
# 这些标志需要在编译和链接阶段都使用。
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

# ===== OpenCV 设置 =====
# 指定 OpenCV 在目标平台上的 CMake 配置路径并查找依赖。
if(NOT OpenCV_DIR AND EXISTS "/home/qy/opencv-4.11.0/loongson/install/lib/cmake/opencv4/OpenCVConfig.cmake")
    set(OpenCV_DIR "/home/qy/opencv-4.11.0/loongson/install/lib/cmake/opencv4")
endif()
find_package(OpenCV REQUIRED)

# ===== 编译器标准 =====
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===== 源文件收集 =====
# 递归收集项目源码文件。
file(GLOB_RECURSE SOURCES 
    "${PROJECT_SOURCE_DIR}/src/**/*.cpp" 
    "${PROJECT_SOURCE_DIR}/src/**/*.h"
    "${PROJECT_SOURCE_DIR}/src/**/*.c")
list(REMOVE_ITEM SOURCES "${PROJECT_SOURCE_DIR}/src/main.cpp")

# ===== 包含头文件目录 =====
# 将 `src` 以及其子目录加入头文件搜索路径。
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/src/Experiment/Baselines)
file(GLOB_RECURSE SUBDIRS ${PROJECT_SOURCE_DIR}/src/*)
foreach(DIR IN LISTS SUBDIRS)
    if(IS_DIRECTORY ${DIR})
        include_directories(${DIR})
    endif()
endforeach()

# ===== 可执行目标 =====
# 生成最终可执行程序。
add_executable(${PROJECT_NAME} src/main.cpp ${SOURCES})

# ===== 链接库 =====
# 链接 OpenCV 与 pthread。
target_link_libraries(${PROJECT_NAME} PRIVATE 
    ${OpenCV_LIBS}
    pthread
    stdc++fs
)

# 旧车本机已经安装 OpenCV 4.11，运行时直接由系统动态链接器查找。
# 不再写死新车 /home/root/qy/opencv411/lib 的 RPATH。

add_executable(zebra_unit_tests tests/zebra_unit_tests.cpp src/ZebraTest/ZebraTest.cpp src/ZebraTest/ZebraDetector.cpp)
target_link_libraries(zebra_unit_tests PRIVATE ${OpenCV_LIBS})

# Standalone sysfs checks can also be compiled on a host without OpenCV.
add_executable(zebra_sysfs_tests tests/zebra_sysfs_tests.cpp src/Contral/PWM/PwmController.cpp src/Contral/GPIO/GPIO.cpp src/ZebraTest/IoStatus.cpp)
target_link_libraries(zebra_sysfs_tests PRIVATE pthread stdc++fs)
add_executable(zebra_hardware_tests tests/zebra_hardware_tests.cpp
    src/Contral/Motor/Motor.cpp src/Contral/Steer/Steer.cpp
    src/Contral/PWM/PwmController.cpp src/Contral/GPIO/GPIO.cpp src/ZebraTest/IoStatus.cpp
    src/Contral/PID/PID.c src/Other/Log/Log.cpp)
target_link_libraries(zebra_hardware_tests PRIVATE ${OpenCV_LIBS} pthread stdc++fs)
foreach(test_target zebra_unit_tests zebra_sysfs_tests zebra_hardware_tests)
    target_compile_options(${test_target} PRIVATE -UNDEBUG)
endforeach()
