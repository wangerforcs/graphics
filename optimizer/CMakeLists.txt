cmake_minimum_required(VERSION 3.5)

project(optimizer VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(SOURCES
	# 在这里添加其他源文件相对于项目根目录的完整路径和名称
	main.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME}
	# 如果你的头文件和源文件不在同一个路径下，可以在这里添加 include 搜索路径
	# 第三方库的头文件全部在 deps/ 下，不需要额外添加搜索路径了
	PRIVATE deps
)

target_compile_definitions(${PROJECT_NAME}
	PRIVATE SPDLOG_FMT_EXTERNAL
	PRIVATE FMT_HEADER_ONLY
)
