cmake_minimum_required(VERSION 3.14)
project(unitree_guide)

set(ROBOT_TYPE Go1)         # The type of robot, support Go1 and A1 currently
set(PLATFORM amd64)         # The platform to compile, support amd64 and arm64

set(CATKIN_MAKE ON)        # Use CATKIN_MAKE or not, ON or OFF
set(SIMULATION ON)      # Use Gazebo or not, ON or OFF
set(REAL_ROBOT OFF)          # Link real robot or not, ON or OFF
set(DEBUG OFF)              # Use debug functions or not, ON or OFF
set(MOVE_BASE OFF)               # Need move_base or not, ON or OFF 

if(NOT DEFINED ROBOT_TYPE)
    message(FATAL_ERROR "[CMake ERROR] Have not defined ROBOT_TYPE")
endif()

if(NOT DEFINED PLATFORM)
    message(FATAL_ERROR "[CMake ERROR] Have not defined PLATFORM")
endif()

if(${ROBOT_TYPE} STREQUAL "A1")         ##STREQUAL 严格等于
    add_definitions(-DROBOT_TYPE_A1)        ##-D是宏定义的标志  ROBOT_TYPE_A1宏会被定义
elseif(${ROBOT_TYPE} STREQUAL "Go1")    ##
    add_definitions(-DROBOT_TYPE_Go1)
else()
    message(FATAL_ERROR "[CMake ERROR] The ROBOT_TYPE is error")
endif()

if(((SIMULATION) AND (REAL_ROBOT)) OR ((NOT SIMULATION) AND (NOT REAL_ROBOT)))      ##同时为真。同时为假，则if成立
    message(FATAL_ERROR "[CMake ERROR] The SIMULATION and REAL_ROBOT can only be one ON one OFF")
endif()

if(SIMULATION OR MOVE_BASE)
    add_definitions(-DRUN_ROS)      ##RUN_ROS默认值为1，但是值没有意义，仅作为一个标志。
    set(CATKIN_MAKE ON)
endif()

set(CMAKE_CXX_STANDARD 11)      ##设置c++11标准来编译
find_package(Boost)         ##寻找boost库。

if(CATKIN_MAKE)
    add_definitions(-DCOMPILE_WITH_ROS)     ##依旧是宏定义，作为标志
    if(MOVE_BASE)
        add_definitions(-DCOMPILE_WITH_MOVE_BASE)
    endif()
endif()

if(DEBUG)
    add_definitions(-DCOMPILE_DEBUG)
    find_package(Python2 COMPONENTS Interpreter Development NumPy)
endif()
## COMPONENTS Interpreter Development 用于查找需要的组建

if(CATKIN_MAKE)
    if(SIMULATION)
        add_definitions(-DCOMPILE_WITH_SIMULATION)
        find_package(catkin REQUIRED COMPONENTS
            controller_manager
            joint_state_controller
            gazebo_ros
            # gazebo
        )
    endif()



    find_package(catkin REQUIRED COMPONENTS     #required后包是必须的，缺少会报错
        genmsg
        robot_state_publisher
        roscpp
        rospy
        std_msgs
        tf
        geometry_msgs
        unitree_legged_msgs
        message_generation
        dynamic_reconfigure
        
    )
    add_message_files(      ##添加消息文件
        FILES
        RISE.msg
        ESO.msg
        L1_adaptive.msg
    )
    
    generate_messages(
        DEPENDENCIES        ##指定依赖项
        std_msgs
    )
    #add dynamic reconfigure api
# generate_dynamic_reconfigure_options(  
#     cfg/Tutorials.cfg  
#     #...
#     )
    catkin_package(
        CATKIN_DEPENDS      ##catkin的依赖项
        unitree_legged_msgs 
        roscpp
        std_msgs
        message_runtime
    )
endif()


include_directories(        #用于向编译器指定需要包含的头文件目录。
    include
    ${Boost_INCLUDE_DIRS}   #通过findpackage保存了boost的头文件路径
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -O3")
#编译器会使用CMAKE_CXX_FLAGS中的默认选项以及额外
#的Gazebo相关选项${GAZEBO_CXX_FLAGS}和优化选项-O3来编译C++源代码。

if(REAL_ROBOT)
    add_definitions(-DCOMPILE_WITH_REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "A1")
        include_directories(        ##添加头文件搜索路径
            library/unitree_legged_sdk_3.2/include
        )
        link_directories(           ##添加库文件搜索路径
            library/unitree_legged_sdk_3.2/lib
        )
    elseif(${ROBOT_TYPE} STREQUAL "Go1")
        include_directories(
            library/unitree_legged_sdk-3.8.0/include
        )
        if(${PLATFORM} STREQUAL "amd64")    ##x64，AMD和；intel都支持此架构
            link_directories(
                library/unitree_legged_sdk-3.8.0/lib/cpp/amd64
            )
        elseif(${PLATFORM} STREQUAL "arm64")    ##一般用于移动设备、嵌入式系统
            link_directories(
                library/unitree_legged_sdk-3.8.0/lib/cpp/arm64
            )
        endif()
    endif()
endif()

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

# aux_source_directory(src SRC_LIST)
file(GLOB_RECURSE SRC_LIST  
    "src/*/*.cpp"
    "src/*/*.cc"
)
#找到src下所有的.cpp .cc文件，储存在src_list中

# # for dynamic reconfigure
# add_executable(dynamic_reconfigure_node src/server.cpp)
# # make sure configure headers are built before any node using them
# add_dependencies(dynamic_reconfigure_node ${PROJECT_NAME}_gencfg)
# # for dynamic  reconfigure
# target_link_libraries(dynamic_reconfigure_node ${catkin_LIBRARIES})


add_executable(junior_ctrl src/main.cpp ${SRC_LIST})    ##SRC_LIST作为构建该执行文件的源文件列表
if(CATKIN_MAKE)
    target_link_libraries(junior_ctrl ${catkin_LIBRARIES})
    add_dependencies(junior_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()
if(DEBUG)
    target_include_directories(junior_ctrl PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
    target_link_libraries(junior_ctrl Python2::Python Python2::NumPy)
endif()
if(REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "A1")
        if(${PLATFORM} STREQUAL "amd64")
            target_link_libraries(junior_ctrl libunitree_legged_sdk_amd64.so)
        elseif(${PLATFORM} STREQUAL "arm64")
            target_link_libraries(junior_ctrl libunitree_legged_sdk_arm64.so)
        endif()
    elseif(${ROBOT_TYPE} STREQUAL "Go1")
            target_link_libraries(junior_ctrl libunitree_legged_sdk.a)
    endif()
endif()
target_link_libraries(junior_ctrl -pthread lcm)

if(NOT CATKIN_MAKE)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
endif()