cmake_minimum_required(VERSION 3.10)
set(CMAKE_BUILD_TYPE RELEASE) #RELEASE DEBUG

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")  

project(MCTS_PARALLE_PLANNING)

# Set C++ standard to C++11
# 指定 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)




# 时间分析
# file(GLOB_RECURSE PROJECT_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")


# 显式指定 Python 解释器路径：如果你已经安装了适当版本的 Python，但 CMake 仍然无法找到它，请使用 PYTHON_EXECUTABLE 变量手动指定 Python 解释器的路径。
set(PYTHON_EXECUTABLE "/usr/bin/python3.8")
# 寻找指定版本的 Python 解释器
find_package(PythonInterp 3.8 REQUIRED)
# 输出找到的 Python 解释器版本
message(STATUS "Found Python interpreter: ${PYTHON_VERSION_STRING}")

# pybind11
# find_package(pybind11 REQUIRED)  # or `add_subdirectory(pybind11)`
# if (PYBIND11_INCLUDE_DIR)
# 	message(STATUS "Found pybind11")
# 	include_directories("${PYBIND11_INCLUDE_DIR}")	#包含Eigen库头文件
# endif()


# # Find MPI package
# find_package(MPI REQUIRED)
# if (MPI_INCLUDE_PATH)
# 	message(STATUS "Found MPI")
# 	MESSAGE(STATUS "MPI found at: ${MPI_CXX_INCLUDE_DIRS}")
# 	include_directories(${MPI_INCLUDE_PATH})	#包含MPI库头文件
# endif()
# 查找 MPI 包
find_package(MPI REQUIRED)
# 包含 MPI 头文件
include_directories(${MPI_INCLUDE_PATH})

# Find Eigen3 package
find_package(Eigen3 REQUIRED)
# Add Eigen3 include directories to include path
# find_package(Eigen3 REQUIRED)
if (EIGEN3_INCLUDE_DIR)
	message(STATUS "Found Eigen3")
	include_directories("${EIGEN3_INCLUDE_DIR}")	#包含Eigen库头文件
endif()

# Find the grid_map package
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
# 包含 MyLibrary 的头文件路径
include_directories(${grid_map_msgs_INCLUDE_DIRS})
include_directories(${grid_map_core_INCLUDE_DIRS})



# 添加自定义头文件
include_directories(${PROJECT_SOURCE_DIR}/planFile/include)
# include_directories(${PROJECT_SOURCE_DIR}/grid_map-master/grid_map_core/include)
# 添加库文件
add_subdirectory (${PROJECT_SOURCE_DIR}/planFile)
# add_subdirectory(${PROJECT_SOURCE_DIR}/grid_map-master/grid_map_core)



# 设置库和可执行文件的输出路径
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
## 找到ROS依赖项并设置
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)


## 指定包含的头文件目录
include_directories(${catkin_INCLUDE_DIRS})


	
add_executable(parallelMCTS_VirtualLoss parallelMCTS_VirtualLoss.cpp)
target_link_libraries(parallelMCTS_VirtualLoss 
	${MPI_LIBRARIES} 
	${grid_map_core_LIBRARIES} 
	${catkin_LIBRARIES} 
	planFile)

