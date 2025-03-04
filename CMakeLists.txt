cmake_minimum_required(VERSION 3.0.2)
project(visual_navigation)
set(CMAKE_CXX_STANDARD 17)


# находим необходимые пакеты для сборки
# компоненты ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  mavros_msgs
  sensor_msgs
  cv_bridge
)
find_package(Eigen3 3.3.7 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp 
)

# указываем в проекте все дериктории с заголовочными файлами
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# добавляем библиотеки для сборки
# библиотека систем управления
#add_library(control_sys_lib STATIC src/uav_controller.cpp)

# добавляем исполняемые файлы
# нода для запуска навигационной системы
add_executable(uav_circle_fly_example src/uav_circle_fly_example.cpp)
add_executable(visual_navigation_node src/visual_navigation_node.cpp)
# пример с полетом БЛА(взаимодействие с аппаратом, управление целевым положением)
#add_executable(go_to_example examples/simple_flight_example.cpp)

# libraris link for visual navigation system
#target_link_libraries(control_sys_lib  ${catkin_LIBRARIES})
target_link_libraries(visual_navigation_node PRIVATE Eigen3::Eigen ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries(uav_circle_fly_example ${catkin_LIBRARIES})
#target_link_libraries(uav_controller_node ${catkin_LIBRARIES})
#target_link_libraries(go_to_example  ${catkin_LIBRARIES})

