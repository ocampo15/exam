# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Sistemasentiempo/esp-idf/components/bootloader/subproject"
  "D:/UN/sistemas_en_tiempo_real/examen/build/bootloader"
  "D:/UN/sistemas_en_tiempo_real/examen/build/bootloader-prefix"
  "D:/UN/sistemas_en_tiempo_real/examen/build/bootloader-prefix/tmp"
  "D:/UN/sistemas_en_tiempo_real/examen/build/bootloader-prefix/src/bootloader-stamp"
  "D:/UN/sistemas_en_tiempo_real/examen/build/bootloader-prefix/src"
  "D:/UN/sistemas_en_tiempo_real/examen/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/UN/sistemas_en_tiempo_real/examen/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/UN/sistemas_en_tiempo_real/examen/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
