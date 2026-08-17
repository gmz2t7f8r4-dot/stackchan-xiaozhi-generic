# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Keep the generated project ELF compile command below Windows' command-line
# limit.  This project enables many optional components, so the accumulated
# include paths otherwise make CreateProcess fail before the compiler starts.
set(CMAKE_C_USE_RESPONSE_FILE_FOR_INCLUDES ON)
set(CMAKE_CXX_USE_RESPONSE_FILE_FOR_INCLUDES ON)

# Add this line to disable the specific warning
add_compile_options(-Wno-missing-field-initializers)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)

set(PROJECT_VER "2.2.6")
project(xiaozhi)
