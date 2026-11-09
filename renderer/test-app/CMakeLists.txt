cmake_minimum_required(VERSION 3.26.0)
project(TestApp)

set(CMAKE_CXX_STANDARD 20)

# Test application executable
add_executable(${PROJECT_NAME}
        src/main.cpp
        src/BulletSoftBodyScene.h
        src/BulletSoftBodyScene.cpp
        src/XpbdSoftBodyApi.h
        src/XpbdSoftBodyApi.cpp
        src/XpbdSoftBodyScene.h
        src/XpbdSoftBodyScene.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
        RendererLib
        BulletSoftBodyAdapter
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
        XPBD_ADAPTER_SOURCE_DLL="${XPBD_ADAPTER_DLL}"
)

# Copy shaders to test app output directory
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/assets"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>/assets"
        COMMENT "Copying assets to test app output directory"
)

foreach(_bullet_dll IN LISTS BULLET_RUNTIME_DLLS)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_bullet_dll}"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
        COMMENT "Copying Bullet runtime DLL"
    )
endforeach()

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${XPBD_ADAPTER_DLL}"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
        COMMENT "Copying XPBD runtime DLL"
)
