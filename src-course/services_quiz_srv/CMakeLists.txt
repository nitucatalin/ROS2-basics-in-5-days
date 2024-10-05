cmake_minimum_required(VERSION 3.8)
project(services_quiz_srv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # Adăugat pentru generarea serviciilor

# Inregistrează fișierul srv pentru generarea codului de mesaje
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Turn.srv"
)

# Dacă vrei să adaugi alte dependențe, le pui aici.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
