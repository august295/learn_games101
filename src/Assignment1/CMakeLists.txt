set(ProjectName "Assignment1")

add_executable(${ProjectName} main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)
target_link_libraries(${ProjectName} ${OpenCV_LIBS})

# cmake3.15
set_target_properties(${ProjectName} PROPERTIES
    DEBUG_POSTFIX "d"
    VS_DEBUGGER_WORKING_DIRECTORY "$(OutDir)"
    MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
