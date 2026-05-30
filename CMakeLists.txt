cmake_minimum_required(VERSION 3.10)
project(character-plugin)

set(CMAKE_CXX_STANDARD 17)

# Исходники
set(SOURCES
    src/StudentController.cpp
    src/PhysicsEngine.h
    src/PIPDController.h
    src/AnimationSampler.h
)

# Исполняемый файл плагина
add_library(character_plugin SHARED ${SOURCES})

# ВАЖНО: добавляем корень проекта в пути поиска заголовков
target_include_directories(character_plugin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Тест
add_executable(standalone_test tests/standalone_test.cpp src/StudentController.cpp)
target_include_directories(standalone_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
