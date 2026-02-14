# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "C:/Users/Artem/Documents/PlatformIO/Projects/Hearing aids ESP32/build/_deps/arduino_emulator-src")
  file(MAKE_DIRECTORY "C:/Users/Artem/Documents/PlatformIO/Projects/Hearing aids ESP32/build/_deps/arduino_emulator-src")
endif()
file(MAKE_DIRECTORY
  "C:/Users/Artem/Documents/PlatformIO/Projects/Hearing aids ESP32/build/_deps/arduino_emulator-build"
  "C:/Users/Artem/Documents/PlatformIO/Projects/Hearing aids ESP32/build/_deps/arduino_emulator-subbuild/arduino_emulator-populate-prefix"
  "C:/Users/Artem/Documents/PlatformIO/Projects/Hearing aids ESP32/build/_deps/arduino_emulator-subbuild/arduino_emulator-populate-prefix/tmp"
  "C:/Users/Artem/Documents/PlatformIO/Projects/Hearing aids ESP32/build/_deps/arduino_emulator-subbuild/arduino_emulator-populate-prefix/src/arduino_emulator-populate-stamp"
  "C:/Users/Artem/Documents/PlatformIO/Projects/Hearing aids ESP32/build/_deps/arduino_emulator-subbuild/arduino_emulator-populate-prefix/src"
  "C:/Users/Artem/Documents/PlatformIO/Projects/Hearing aids ESP32/build/_deps/arduino_emulator-subbuild/arduino_emulator-populate-prefix/src/arduino_emulator-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/Artem/Documents/PlatformIO/Projects/Hearing aids ESP32/build/_deps/arduino_emulator-subbuild/arduino_emulator-populate-prefix/src/arduino_emulator-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/Artem/Documents/PlatformIO/Projects/Hearing aids ESP32/build/_deps/arduino_emulator-subbuild/arduino_emulator-populate-prefix/src/arduino_emulator-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
