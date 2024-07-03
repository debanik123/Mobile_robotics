cmake_minimum_required(VERSION 3.1)
project(eigen_test)

# Set the path to the Eigen library
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Add the Eigen include directory
# include_directories(${EIGEN_PATH})

# Add your source files
add_executable(eigen_test eigen.cpp)

# Link any additional libraries if needed
target_link_libraries(eigen_test Eigen3::Eigen)
install(TARGETS
  eigen_test
  DESTINATION lib/${PROJECT_NAME})

