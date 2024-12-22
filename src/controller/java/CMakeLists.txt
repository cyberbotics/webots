# Find SWIG
find_package(SWIG)

# Check if SWIG was found
if(SWIG_FOUND)
    include(UseSWIG)
    message(STATUS "SWIG found: ${SWIG_EXECUTABLE}")

    # Set the module name (should match %module in the .i file)
    set(SWIG_MODULE_NAME wrapper)

    # Set the output directory for generated Java files
    set(SWIG_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/SWIG_generated_files)

    # Set the package name for the generated Java code
    set(SWIG_PACKAGE_NAME com.cyberbotics.webots.controller)

    # Set the input SWIG interface file
    set(SWIG_INTERFACE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/controller.i)

    # Set include directories for C++ headers
    set(SWIG_INCLUDE_DIRS
            ${CMAKE_SOURCE_DIR}/include/controller/cpp
            ${CMAKE_SOURCE_DIR}/include/controller/c
            )

    # Create the output directory if it doesn't exist
    file(MAKE_DIRECTORY ${SWIG_OUTPUT_DIR})

    # Configure SWIG
    set_source_files_properties(${SWIG_INTERFACE_FILE}
            PROPERTIES CPLUSPLUS ON)
    set_source_files_properties(${SWIG_INTERFACE_FILE}
            PROPERTIES SWIG_FLAGS "-c++;-package;${SWIG_PACKAGE_NAME};-outdir;${SWIG_OUTPUT_DIR}")

    # Generate the SWIG wrapper code
    swig_add_library(${SWIG_MODULE_NAME} TYPE SHARED
            LANGUAGE java
            SOURCES ${SWIG_INTERFACE_FILE})
    set_property(TARGET ${SWIG_MODULE_NAME} PROPERTY SWIG_USE_TARGET_INCLUDE_DIRECTORIES TRUE)
    swig_link_libraries(${SWIG_MODULE_NAME}
            PRIVATE
            Controller
            CppController
            )

    target_include_directories(${SWIG_MODULE_NAME}
            PUBLIC
            ${SWIG_INCLUDE_DIRS}
            )

    # Get the SWIG generated source file name
    get_property(SWIG_WRAPPER_SOURCE TARGET ${SWIG_MODULE_NAME} PROPERTY SWIG_GENERATED_SOURCE_NAME_JAVA)

    # --- Compilation of Generated Java Code ---
    # Create a list of Java source files (including generated ones)
    file(GLOB_RECURSE JAVA_SOURCES LIST_DIRECTORIES false
            "${SWIG_OUTPUT_DIR}/*.java"
            "${CMAKE_CURRENT_SOURCE_DIR}/*.java")

    # Remove some of the auto-generated SWIG files which are not needed
    list(FILTER JAVA_SOURCES EXCLUDE REGEX ".*(swigjni|Swig).*")

    # Compile Java source files into a JAR file
    set(JAR_OUTPUT_PATH ${WEBOTS_CONTROLLER_LIB_PATH}/java/Controller.jar)
    file(MAKE_DIRECTORY ${WEBOTS_CONTROLLER_LIB_PATH}/java)

    # Add custom command to compile Java files after SWIG generation
    add_custom_command(
            OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${SWIG_PACKAGE_NAME}
            COMMAND javac --release 11 ${JAVA_SOURCES} -d ${CMAKE_CURRENT_BINARY_DIR}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            DEPENDS ${SWIG_MODULE_NAME}
            COMMENT "Compiling Java sources"
            VERBATIM
    )

    # Add custom command to create JAR after Java compilation
    add_custom_command(
            OUTPUT ${JAR_OUTPUT_PATH}
            COMMAND jar cfv ${JAR_OUTPUT_PATH} ${SWIG_PACKAGE_NAME}/*.class
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${SWIG_PACKAGE_NAME}
            COMMENT "Creating JAR file"
            VERBATIM
    )

    # Add a custom target to trigger the build of the java library
    add_custom_target(java_controller
            DEPENDS ${JAR_OUTPUT_PATH}
            )
    # --- Compilation of Native Library ---

    # OS-specific settings
    if(WIN32)
        # Windows
        set(JNI_LIBRARY_NAME JavaController)
        set(JNI_LIBRARY_OUTPUT_PATH ${WEBOTS_CONTROLLER_LIB_PATH}/java/${JNI_LIBRARY_NAME}.dll)
        set(JNI_LINK_FLAGS "-shared -mwindows -Wl,--add-stdcall-alias -Wl,--enable-auto-import -O")

        # Find Java include directories (prefer JAVA_HOME, then registry)
        if(DEFINED ENV{JAVA_HOME})
            find_path(JAVA_INCLUDE_PATH jni.h
                    PATHS
                    "$ENV{JAVA_HOME}/include"
                    )

            find_path(JAVA_MD_INCLUDE_PATH jni_md.h
                    PATHS
                    "$ENV{JAVA_HOME}/include/win32"
                    )
        else()
            # Query the Windows Registry for Java installations
            # 64-bit Java
            file(READ "/HKEY_LOCAL_MACHINE/SOFTWARE/JavaSoft/Java Development Kit" JAVA_KEYS_64)
            string(REPLACE "\n" ";" JAVA_KEYS_64 ${JAVA_KEYS_64})
            foreach(JAVA_KEY ${JAVA_KEYS_64})
              file(READ "/HKEY_LOCAL_MACHINE/SOFTWARE/JavaSoft/Java Development Kit/${JAVA_KEY}" JAVA_DATA_64)
              string(REPLACE "\n" ";" JAVA_DATA_64 ${JAVA_DATA_64})
              list(FIND JAVA_DATA_64 "JavaHome" JAVA_HOME_INDEX_64)
              if(NOT ${JAVA_HOME_INDEX_64} EQUAL -1)
                math(EXPR JAVA_HOME_INDEX_64 "${JAVA_HOME_INDEX_64} + 1")
                list(GET JAVA_DATA_64 ${JAVA_HOME_INDEX_64} JAVA_HOME_PATH_64)
                string(REPLACE "\\" "/" JAVA_HOME_PATH_64 ${JAVA_HOME_PATH_64})
                find_path(JAVA_INCLUDE_PATH jni.h PATHS "${JAVA_HOME_PATH_64}/include")
                find_path(JAVA_MD_INCLUDE_PATH jni_md.h PATHS "${JAVA_HOME_PATH_64}/include/win32")
                if(JAVA_INCLUDE_PATH AND JAVA_MD_INCLUDE_PATH)
                  break()
                endif()
              endif()
            endforeach()

            # 32-bit Java (on 64-bit Windows)
            if(CMAKE_SIZEOF_VOID_P EQUAL 8 AND (NOT JAVA_INCLUDE_PATH OR NOT JAVA_MD_INCLUDE_PATH))
              file(READ "/HKEY_LOCAL_MACHINE/SOFTWARE/Wow6432Node/JavaSoft/Java Development Kit" JAVA_KEYS_32)
              string(REPLACE "\n" ";" JAVA_KEYS_32 ${JAVA_KEYS_32})
              foreach(JAVA_KEY ${JAVA_KEYS_32})
                file(READ "/HKEY_LOCAL_MACHINE/SOFTWARE/Wow6432Node/JavaSoft/Java Development Kit/${JAVA_KEY}" JAVA_DATA_32)
                string(REPLACE "\n" ";" JAVA_DATA_32 ${JAVA_DATA_32})
                list(FIND JAVA_DATA_32 "JavaHome" JAVA_HOME_INDEX_32)
                if(NOT ${JAVA_HOME_INDEX_32} EQUAL -1)
                  math(EXPR JAVA_HOME_INDEX_32 "${JAVA_HOME_INDEX_32} + 1")
                  list(GET JAVA_DATA_32 ${JAVA_HOME_INDEX_32} JAVA_HOME_PATH_32)
                  string(REPLACE "\\" "/" JAVA_HOME_PATH_32 ${JAVA_HOME_PATH_32})
                  find_path(JAVA_INCLUDE_PATH jni.h PATHS "${JAVA_HOME_PATH_32}/include")
                  find_path(JAVA_MD_INCLUDE_PATH jni_md.h PATHS "${JAVA_HOME_PATH_32}/include/win32")
                  if(JAVA_INCLUDE_PATH AND JAVA_MD_INCLUDE_PATH)
                    break()
                  endif()
                endif()
              endforeach()
            endif()
        endif()

    elseif(APPLE)
        # macOS
        set(JNI_LIBRARY_NAME libJavaController.jnilib)
        set(JNI_LIBRARY_OUTPUT_PATH ${WEBOTS_CONTROLLER_LIB_PATH}/java/${JNI_LIBRARY_NAME})
        set(JNI_LINK_FLAGS "-dynamiclib -install_name @rpath/Contents/lib/controller/java/libJavaController.jnilib -Wl,-rpath,@loader_path/../../../..")

        # Find Java include directories (prefer JAVA_HOME, then system paths)
        if(DEFINED ENV{JAVA_HOME})
            find_path(JAVA_INCLUDE_PATH jni.h
                    PATHS
                    "$ENV{JAVA_HOME}/include"
                    )

            find_path(JAVA_MD_INCLUDE_PATH jni_md.h
                    PATHS
                    "$ENV{JAVA_HOME}/include/darwin"
                    )
        else()
            # Use 'which javac' to find the Java installation
            execute_process(
                    COMMAND which javac
                    OUTPUT_VARIABLE JAVAC_PATH
                    OUTPUT_STRIP_TRAILING_WHITESPACE
            )
            get_filename_component(JAVA_BIN_DIR ${JAVAC_PATH} DIRECTORY)
            get_filename_component(JAVA_HOME_GUESS ${JAVA_BIN_DIR} DIRECTORY)

            find_path(JAVA_INCLUDE_PATH jni.h
                    PATHS
                    "${JAVA_HOME_GUESS}/include"
                    "/Library/Java/JavaVirtualMachines/*/Contents/Home/include"  # Common macOS location
                    "/System/Library/Frameworks/JavaVM.framework/Headers"
            )
            find_path(JAVA_MD_INCLUDE_PATH jni_md.h
                    PATHS
                    "${JAVA_HOME_GUESS}/include/darwin"
                    "/Library/Java/JavaVirtualMachines/*/Contents/Home/include/darwin"
                    "/System/Library/Frameworks/JavaVM.framework/Headers"
            )
        endif()

    else()
        # Linux
        set(JNI_LIBRARY_NAME libJavaController.so)
        set(JNI_LIBRARY_OUTPUT_PATH ${WEBOTS_CONTROLLER_LIB_PATH}/java/${JNI_LIBRARY_NAME})
        set(JNI_LINK_FLAGS "-shared -fPIC")

        # Find Java include directories (prefer JAVA_HOME, then system paths)
        if(DEFINED ENV{JAVA_HOME})
            find_path(JAVA_INCLUDE_PATH jni.h
                    PATHS
                    "$ENV{JAVA_HOME}/include"
                    )

            find_path(JAVA_MD_INCLUDE_PATH jni_md.h
                    PATHS
                    "$ENV{JAVA_HOME}/include/linux"
                    )
        endif()
    endif()

    # Fallback: If JAVA_HOME is not defined or not found, use javac to find Java include paths
    if(NOT JAVA_INCLUDE_PATH OR NOT JAVA_MD_INCLUDE_PATH)
        # Use 'which javac' to find the Java installation
        execute_process(
                COMMAND which javac
                OUTPUT_VARIABLE JAVAC_PATH
                OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        if(JAVAC_PATH)
            # Extract JAVA_HOME from javac path
            get_filename_component(JAVA_BIN_DIR ${JAVAC_PATH} DIRECTORY)
            get_filename_component(JAVA_HOME_GUESS ${JAVA_BIN_DIR} DIRECTORY)  # Correctly go up one level for JAVA_HOME

            # Search for jni.h
            find_path(JAVA_INCLUDE_PATH jni.h
                    PATHS
                    "${JAVA_HOME_GUESS}/include"
                    "/usr/lib/jvm/*/include"        # Common Debian/Ubuntu locations
                    "/usr/java/*/include"           # Common RedHat/Fedora locations
                    "/usr/include/java"
            )

            # OS-specific search for jni_md.h
            if(WIN32)
                find_path(JAVA_MD_INCLUDE_PATH jni_md.h
                        PATHS
                        "${JAVA_HOME_GUESS}/include/win32"
                )
            elseif(APPLE)
                find_path(JAVA_MD_INCLUDE_PATH jni_md.h
                        PATHS
                        "${JAVA_HOME_GUESS}/include/darwin"
                        "/Library/Java/JavaVirtualMachines/*/Contents/Home/include/darwin"
                        "/System/Library/Frameworks/JavaVM.framework/Headers"
                )
            else()  # Linux
                find_path(JAVA_MD_INCLUDE_PATH jni_md.h
                        PATHS
                        "${JAVA_HOME_GUESS}/include/linux"
                        "/usr/lib/jvm/*/include/linux"
                        "/usr/java/*/include/linux"
                        "/usr/include/java/linux"
                )
            endif()
        else()
            message(WARNING "Could not find 'javac'. Java include paths may not be found.")
        endif()
    endif()
 

    target_include_directories(${SWIG_MODULE_NAME}
        PUBLIC
        ${JAVA_INCLUDE_PATH}  # Add this line
        ${JAVA_MD_INCLUDE_PATH} # Add this line
        )


    # Check if Java include directories were found
    if(NOT JAVA_INCLUDE_PATH OR NOT JAVA_MD_INCLUDE_PATH)
        message(FATAL_ERROR "Java include directories not found. Please ensure Java is properly installed.")
    else()
        message(STATUS "JAVA_INCLUDE_PATH: ${JAVA_INCLUDE_PATH}")
        message(STATUS "JAVA_MD_INCLUDE_PATH: ${JAVA_MD_INCLUDE_PATH}")
    endif()

    # Generate the JNI library in a separate target
    add_library(${JNI_LIBRARY_NAME} SHARED ${SWIG_WRAPPER_SOURCE})
    add_dependencies(${JNI_LIBRARY_NAME} ${SWIG_MODULE_NAME})

    target_include_directories(${JNI_LIBRARY_NAME}
            PUBLIC
            ${JAVA_INCLUDE_PATH}
            ${JAVA_MD_INCLUDE_PATH}
            ${SWIG_INCLUDE_DIRS}
            )

    target_link_libraries(${JNI_LIBRARY_NAME}
            PRIVATE
            Controller
            CppController
            )

    set_target_properties(${JNI_LIBRARY_NAME} PROPERTIES
            PREFIX ""
            OUTPUT_NAME ${JNI_LIBRARY_NAME}
            LIBRARY_OUTPUT_DIRECTORY ${JNI_LIBRARY_OUTPUT_PATH}
            )

    # Set linker flags for the JNI library
    set_target_properties(${JNI_LIBRARY_NAME} PROPERTIES LINK_FLAGS ${JNI_LINK_FLAGS})

    # Add dependency on the SWIG module
    add_dependencies(java_controller ${JNI_LIBRARY_NAME})

    # Create a symbolic link or copy the JNI library to the output directory
    # This simplifies loading the library from Java code
    add_custom_command(TARGET ${JNI_LIBRARY_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E create_symlink
            ${JNI_LIBRARY_OUTPUT_PATH}
            ${CMAKE_CURRENT_BINARY_DIR}/${JNI_LIBRARY_NAME}
            COMMENT "Creating symbolic link for ${JNI_LIBRARY_NAME}"
    )

else()
    message(WARNING "SWIG not found. Skipping Java API generation.")
endif()
