set(ProjectName "Assignment3")

add_executable(${ProjectName} main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp Texture.hpp Texture.cpp Shader.hpp OBJ_Loader.h)
target_link_libraries(${ProjectName} ${OpenCV_LIBS})
#target_compile_options(Rasterizer PUBLIC -Wall -Wextra -pedantic)

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