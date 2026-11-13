cmake_minimum_required(VERSION 3.26.0)
project(BulletSoftBodySmoke)

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/src/include
)
target_link_libraries(${PROJECT_NAME} PRIVATE BulletSoftBodyAdapter)


