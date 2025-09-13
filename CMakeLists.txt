# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
# idf_build_set_property(MINIMAL_BUILD ON)

# SPIFFS data upload configuration
set(SPIFFS_IMAGE_FLASH_IN_PROJECT 1)
set(SPIFFS_IMAGE_DEPENDS "")

project(fortuna-next)
