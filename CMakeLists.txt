cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)

# SPIFFS data upload configuration
set(SPIFFS_IMAGE_FLASH_IN_PROJECT 1)
set(SPIFFS_IMAGE_DEPENDS "")

project(fortuna-next)
