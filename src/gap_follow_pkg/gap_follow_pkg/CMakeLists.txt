cmake_minimum_required(VERSION 3.8)
project(gap_follow_pkg)

# Paquete Python ───────────────────────────────────────────────────────────────
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

# Instala los launch, cfg, etc.
install(
  DIRECTORY launch resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
