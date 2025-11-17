set(ProjectName "Assignment6")

add_executable(${ProjectName} main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
    Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
    Renderer.cpp Renderer.hpp)

# cmake3.15
set_target_properties(${ProjectName} PROPERTIES
    DEBUG_POSTFIX "d"
    VS_DEBUGGER_WORKING_DIRECTORY "$(OutDir)"
    MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

    # 拷贝资源文件
add_custom_target(
    "${ProjectName}CopyTask" ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/models" "${BUILD_DIR}/bin/${ProjectName}/models"
    COMMENT "Copying resources"
)
set_property(TARGET "${ProjectName}CopyTask" PROPERTY FOLDER "CMakePredefinedTargets")