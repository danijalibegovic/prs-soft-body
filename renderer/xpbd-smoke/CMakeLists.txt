cmake_minimum_required(VERSION 3.26.0)
project(XpbdSoftBodySmoke)

add_executable(${PROJECT_NAME}
    main.cpp
    ${CMAKE_SOURCE_DIR}/test-app/src/XpbdSoftBodyApi.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/src/include
    ${CMAKE_SOURCE_DIR}/test-app/src
)
target_link_libraries(${PROJECT_NAME} PRIVATE spdlog::spdlog)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${XPBD_ADAPTER_DLL}"
        "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
)
