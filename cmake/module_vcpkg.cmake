# 设置 vcpkg 配置
if(CMAKE_HOST_SYSTEM_NAME MATCHES "Windows")
    # 获取 VCPKG_ROOT
    if(NOT DEFINED VCPKG_ROOT)
        if(DEFINED ENV{VCPKG_ROOT})
            set(VCPKG_ROOT "$ENV{VCPKG_ROOT}")
        else()
            set(VCPKG_ROOT "C:/dev/vcpkg")
        endif()
    endif()

    # 设置工具链文件
    set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")
    set(VCPKG_TARGET_TRIPLET "x64-windows-static")
    set(PKG_CONFIG_EXECUTABLE "${VCPKG_ROOT}/installed/${VCPKG_TARGET_TRIPLET}/tools/pkgconf/pkgconf.exe")
    set(PKG_CONFIG_PATH "${VCPKG_ROOT}/installed/${VCPKG_TARGET_TRIPLET}/lib/pkgconfig")

    # 路径输出
    message(STATUS "VCPKG_ROOT: ${VCPKG_ROOT}")
    message(STATUS "CMAKE_TOOLCHAIN_FILE: ${CMAKE_TOOLCHAIN_FILE}")
endif()

macro(VCPKG_LOAD_3RDPARTY)
    ################################################################################
    # Eigen3
    #   vcpkg install eigen3:x64-windows-static
    ################################################################################
    find_package(Eigen3 CONFIG REQUIRED)
    if(Eigen3_FOUND)
        include_directories(${EIGEN3_INCLUDE_DIRS})
        include_directories(${EIGEN3_INCLUDE_DIRS}/../)
    else()
        message(FATAL_ERROR "Eigen3 not found. Please install via: vcpkg install eigen3:${VCPKG_TARGET_TRIPLET}")
    endif()

    ################################################################################
    # OpenCV
    #   vcpkg install opencv:x64-windows-static
    ################################################################################
    find_package(OpenCV CONFIG REQUIRED)
    if(OpenCV_FOUND)
        include_directories(${OpenCV_INCLUDE_DIRS})
    else()
        message(FATAL_ERROR "OpenCV not found. Please install via: vcpkg install opencv:${VCPKG_TARGET_TRIPLET}")
    endif()
endmacro()
