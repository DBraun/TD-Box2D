cmake_minimum_required(VERSION 3.13.0 FATAL_ERROR)

project(TD-Box2D VERSION 0.0.1)
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT TD-Box2D)

################################################################################
# Set target arch type if empty. Visual studio solution generator provides it.
################################################################################
if(NOT CMAKE_VS_PLATFORM_NAME)
    set(CMAKE_VS_PLATFORM_NAME "x64")
endif()
message("${CMAKE_VS_PLATFORM_NAME} architecture in use")

if(NOT ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "x64"))
    message(FATAL_ERROR "${CMAKE_VS_PLATFORM_NAME} arch is not supported!")
endif()

################################################################################
# Global configuration types
################################################################################
set(CMAKE_CONFIGURATION_TYPES
    "Debug"
    "Release"
    CACHE STRING "" FORCE
)

################################################################################
# Global compiler options
################################################################################
if(MSVC)
    # remove default flags provided with CMake for MSVC
    set(CMAKE_CXX_FLAGS "")
    set(CMAKE_CXX_FLAGS_DEBUG "")
    set(CMAKE_CXX_FLAGS_RELEASE "")

endif()

################################################################################
# Global linker options
################################################################################
# if(MSVC)
#     # remove default flags provided with CMake for MSVC
#     set(CMAKE_EXE_LINKER_FLAGS "")
#     set(CMAKE_MODULE_LINKER_FLAGS "")
#     set(CMAKE_SHARED_LINKER_FLAGS "")
#     set(CMAKE_STATIC_LINKER_FLAGS "")
#     set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS}")
#     set(CMAKE_MODULE_LINKER_FLAGS_DEBUG "${CMAKE_MODULE_LINKER_FLAGS}")
#     set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS}")
#     set(CMAKE_STATIC_LINKER_FLAGS_DEBUG "${CMAKE_STATIC_LINKER_FLAGS}")
#     set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS}")
#     set(CMAKE_MODULE_LINKER_FLAGS_RELEASE "${CMAKE_MODULE_LINKER_FLAGS}")
#     set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS}")
#     set(CMAKE_STATIC_LINKER_FLAGS_RELEASE "${CMAKE_STATIC_LINKER_FLAGS}")
# endif()
set(cxx_no_exception_flags "-EHs-c- -D_HAS_EXCEPTIONS=1")

################################################################################
# Nuget packages function stub.
################################################################################
function(use_package TARGET PACKAGE VERSION)
    message(WARNING "No implementation of use_package. Create yours. "
                    "Package \"${PACKAGE}\" with version \"${VERSION}\" "
                    "for target \"${TARGET}\" is ignored!")
endfunction()

################################################################################
# Common utils
################################################################################
include(CMake/Utils.cmake)

################################################################################
# Additional Global Settings(add specific info there)
################################################################################
include(CMake/GlobalSettingsInclude.cmake OPTIONAL)

################################################################################
# Use solution folders feature
################################################################################
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

################################################################################
# Sub-projects
################################################################################
project(TD-Box2D VERSION 0.0.1)

include(CMake/Utils.cmake)

add_library(TD-Box2D SHARED ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/box2d/include )

# Include header directories
target_include_directories(TD-Box2D PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/box2d/include>
    $<INSTALL_INTERFACE:TD-Box2D> )

# Install library
INSTALL(TARGETS TD-Box2D
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} )


# Detect and add SFML
#Find any version 2.X of SFML
#See the FindSFML.cmake file for additional details and instructions
find_package(SFML REQUIRED network audio graphics window system PATHS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/SFML)
if(SFML_FOUND)
  include_directories(${SFML_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} ${SFML_LIBRARIES} ${SFML_DEPENDENCIES})
endif()

add_subdirectory(thirdparty/Box2D)
# set(box2d_DIR "${CMAKE_SOURCE_DIR}/thirdparty/Box2D")
# set(BOX2D_LIBRARY "${BOX2D_DIR}/thirdparty/Box2D/Release/Box2D.lib")
# find_package(box2d REQUIRED)
# target_link_libraries(${PROJECT_NAME} PRIVATE box2d)
# if(SFML_FOUND)
#   include_directories(${SFML_INCLUDE_DIR})
#   target_link_libraries(${PROJECT_NAME} ${SFML_LIBRARIES} ${SFML_DEPENDENCIES})
# endif()

set_target_properties(${PROJECT_NAME} PROPERTIES
                      VS_DEBUGGER_COMMAND "C:\\Program Files\\Derivative\\TouchDesigner\\bin\\TouchDesigner.exe"
                      VS_DEBUGGER_COMMAND_ARGUMENTS "..\\$(ProjectName).toe")
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_compile_options(TD-Box2D PRIVATE /EHsc /GR)

### add all projects for DLLs that can be used inside TouchDesigner
add_subdirectory(TD-Box2D)
