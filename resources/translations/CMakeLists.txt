# Placeholder CMakeLists.txt for resources/translations
# TODO: Convert the logic from resources/translations/Makefile to CMake.
# This will likely involve using Qt's CMake functions for translations (e.g., qt_add_translation).

cmake_minimum_required(VERSION 3.10)
project(webots_resources_translations)

message(STATUS "Configuring resources/translations")

# Example for Qt translations:
# find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core LinguistTools)
#
# set(TS_FILES
#   qt_de.ts
#   qt_es.ts
#   # ... other .ts files
#   wb_de.ts
#   wb_es.ts
#   # ... other .ts files
# )
#
# qt_add_translations(QM_FILES ${TS_FILES})
#
# install(FILES ${QM_FILES} DESTINATION share/webots/translations) # Adjust destination as needed