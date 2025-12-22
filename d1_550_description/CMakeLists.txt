cmake_minimum_required(VERSION 3.8)

project(d1_550_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  config
  launch
  meshes
  urdf
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz config
install(FILES
  urdf.rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/environment_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
