
# Compile your code with: (see README for additional information)
# mkdir build
# cd build
# cmake -DCMAKE_BUILD_TYPE=Release ..
# make

cmake_minimum_required (VERSION 3.9)

project(gridbots_decisions LANGUAGES CXX C)

# Identify the location of the kilosim library
add_subdirectory(submodules/kilosim-gridbots)

# Directory containing header files
include_directories(api)

# Be sure to list all source files
add_executable(gridbots_decisions
  src/Bot.cpp
  src/main.cpp
)

# Link the kilosim library
target_link_libraries(gridbots_decisions PRIVATE kilosim kilosim_gridbots)

target_compile_options(gridbots_decisions PRIVATE -pg -march=native -Wall -Wextra)
# target_compile_options(gridbots_decisions PRIVATE -pg -O0 -march=native -Wall -Wextra)


# Install to bin/ directory
install(TARGETS gridbots_decisions RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/bin OPTIONAL)