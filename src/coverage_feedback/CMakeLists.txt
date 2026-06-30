cmake_minimum_required(VERSION 3.0.2)
project(coverage_feedback)

## Compile as C++17, to match the rest of the workspace (mower_map, mower_logic)
add_compile_options(-std=c++17)

## OpenCV is used for the coverage grid morphology (erode/open), connected
## components, contours and PCA. It is already a workspace dependency (mower_map).
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        visualization_msgs
        message_generation
        mower_map
        mower_msgs
        slic3r_coverage_planner
        xbot_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        CoverageStatus.msg
)

## Generate services in the 'srv' folder
add_service_files(
        FILES
        GetFillPaths.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        nav_msgs
        mower_map
        slic3r_coverage_planner
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        visualization_msgs
        message_runtime
        mower_map
        mower_msgs
        slic3r_coverage_planner
        xbot_msgs
)

###########
## Build ##
###########

## nlohmann/json for the per-pass replay JSON (coverage/planned_path/actual_track/meta), matching how
## mower_map and mower_logic pull it in from the in-tree copy.
add_subdirectory(../lib/json json EXCLUDE_FROM_ALL)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(coverage_feedback src/coverage_feedback_node.cpp)
add_dependencies(coverage_feedback ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
## stdc++fs: std::filesystem (used for on-disk coverage persistence) needs an explicit link on
## GCC < 9; on the GCC 9 toolchain Noetic targets it is a harmless empty stub.
target_link_libraries(coverage_feedback ${catkin_LIBRARIES} ${OpenCV_LIBS} stdc++fs nlohmann_json::nlohmann_json)
