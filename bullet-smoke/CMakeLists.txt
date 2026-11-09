cmake_minimum_required(VERSION 3.26.0)
project(BulletSoftBodySmoke)

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/src/include
)
target_link_libraries(${PROJECT_NAME} PRIVATE BulletSoftBodyAdapter)

foreach(_bullet_dll IN LISTS BULLET_RUNTIME_DLLS)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_bullet_dll}"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
    )
endforeach()
