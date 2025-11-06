set(ProjectName "Assignment5")

add_executable(${ProjectName} main.cpp Object.hpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp Scene.hpp Light.hpp Renderer.cpp)

# cmake3.15
set_target_properties(${ProjectName} PROPERTIES
    DEBUG_POSTFIX "d"
    VS_DEBUGGER_WORKING_DIRECTORY "$(OutDir)"
    MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")