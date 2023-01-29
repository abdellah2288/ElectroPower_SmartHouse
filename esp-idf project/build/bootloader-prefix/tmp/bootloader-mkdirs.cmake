# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/abdellah/esp-idf/components/bootloader/subproject"
  "/home/abdellah/esp-idf/ep_smart_house/build/bootloader"
  "/home/abdellah/esp-idf/ep_smart_house/build/bootloader-prefix"
  "/home/abdellah/esp-idf/ep_smart_house/build/bootloader-prefix/tmp"
  "/home/abdellah/esp-idf/ep_smart_house/build/bootloader-prefix/src/bootloader-stamp"
  "/home/abdellah/esp-idf/ep_smart_house/build/bootloader-prefix/src"
  "/home/abdellah/esp-idf/ep_smart_house/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/abdellah/esp-idf/ep_smart_house/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/abdellah/esp-idf/ep_smart_house/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
