cmake_minimum_required(VERSION 3.22)
# This is the version of cmake on ubuntu 22
project(OpenSimDOMUPlugin)

set(PLUGIN_NAME "osimDOMUPlugin"
    CACHE STRING "Name of shared library to create")

# Set for C++20 language features.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -march=native")
set(GCC_OPT_ENABLE "-funroll-loops")

# Find and hook up to OpenSim.
# ----------------------------
# CPMAddPackage("gh:catchorg/Catch2#v3.8.0")
# CPMAddPackage("gh:casadi/casadi#3.7.0")
# CPMAddPackage("gh:simbody/simbody#Simbody-3.7")
# CPMAddPackage("gh:opensim-org/opensim-core#aa40605bae4ecb81c0bae23638f2cd90f20202e5")
set(OpenSim_DIR "~/opensim-core/cmake")
find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

set(BUILD_SHARED_LIBS "TRUE" CACHE BOOL "Build shared libraries." FORCE)

# Choose common build locations for executables and libraries
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

include_directories(${OPENSIMSIMBODY_INCLUDE_DIRS} ./)
# include_directories(${OPENSIMSIMBODY_INCLUDE_DIRS} ./Simbody/include)

set (Plugin_Simbody_HEADERS
                        Simbody/include/simbody/internal/AssemblyCondition_DistanceSensors.h
)

set (Plugin_Simbody_SOURCES
                        Simbody/src/AssemblyCondition_DistanceSensors.cpp
)

set (Plugin_OpenSim_Analyses_HEADERS
                        OpenSim/Extension/Analyses/osimAnalysesDLL.h
                        OpenSim/Extension/Analyses/RegisterTypes_osimAnalyses.h
                        OpenSim/Extension/Analyses/osimAnalyses.h
                        OpenSim/Extension/Analyses/DistanceKinematics.h
                        OpenSim/Extension/Analyses/DistanceKinematicsIMU.h
)

set (Plugin_OpenSim_Analyses_SOURCES
                        OpenSim/Extension/Analyses/osimAnalysesDLL.cpp
                        OpenSim/Extension/Analyses/RegisterTypes_osimAnalyses.cpp
                        OpenSim/Extension/Analyses/DistanceKinematics.cpp
                        OpenSim/Extension/Analyses/DistanceKinematicsIMU.cpp
)
set (Plugin_OpenSim_Common_HEADERS
                        OpenSim/Extension/Common/osimCommonDLL.h
                        OpenSim/Extension/Common/RegisterTypes_osimCommon.h
                        OpenSim/Extension/Common/osimCommon.h
                        OpenSim/Extension/Common/DistanceBaseDataReader.h
                        OpenSim/Extension/Common/DistanceDataReader.h
                        OpenSim/Extension/Common/DistanceDataReaderSettings.h
                        )

set (Plugin_OpenSim_Common_SOURCES
                        OpenSim/Extension/Common/osimCommonDLL.cpp
                        OpenSim/Extension/Common/RegisterTypes_osimCommon.cpp
                        OpenSim/Extension/Common/DistanceBaseDataReader.cpp
                        OpenSim/Extension/Common/DistanceDataReader.cpp
                        )

set (Plugin_OpenSim_Simulation_HEADERS
                        OpenSim/Extension/Simulation/osimSimulationDLL.h
                        OpenSim/Extension/Simulation/RegisterTypes_osimSimulation.h
                        OpenSim/Extension/Simulation/osimSimulation.h
                        OpenSim/Extension/Simulation/DistancesReference.h
                        OpenSim/Extension/Simulation/InverseKinematicsSolverExt.h
                        OpenSim/Extension/Simulation/OpenSense/IMUPlacerExt.h
                        )

set (Plugin_OpenSim_Simulation_SOURCES
                        OpenSim/Extension/Simulation/osimSimulationDLL.cpp
                        OpenSim/Extension/Simulation/RegisterTypes_osimSimulation.cpp
                        OpenSim/Extension/Simulation/DistancesReference.cpp
                        OpenSim/Extension/Simulation/InverseKinematicsSolverExt.cpp
                        OpenSim/Extension/Simulation/OpenSense/IMUPlacerExt.cpp
                        )

set (Plugin_OpenSim_Tools_HEADERS
                        OpenSim/Extension/Tools/osimToolsDLL.h
                        OpenSim/Extension/Tools/RegisterTypes_osimTools.h
                        OpenSim/Extension/Tools/osimTools.h
                        OpenSim/Extension/Tools/DistanceInverseKinematicsTool.h
                        OpenSim/Extension/Tools/GenericModelMaker.h
                        OpenSim/Extension/Tools/MarkerPair.h
                        OpenSim/Extension/Tools/MarkerPairSet.h
                        OpenSim/Extension/Tools/Measurement.h
                        OpenSim/Extension/Tools/MeasurementSet.h
                        OpenSim/Extension/Tools/ModelScaler.h
                        OpenSim/Extension/Tools/ScaleTool.h
                        )


set (Plugin_OpenSim_Tools_SOURCES
                        OpenSim/Extension/Tools/osimToolsDLL.cpp
                        OpenSim/Extension/Tools/RegisterTypes_osimTools.cpp
                        OpenSim/Extension/Tools/DistanceInverseKinematicsTool.cpp
                        OpenSim/Extension/Tools/GenericModelMaker.cpp
                        OpenSim/Extension/Tools/MarkerPair.cpp
                        OpenSim/Extension/Tools/MarkerPairSet.cpp
                        OpenSim/Extension/Tools/Measurement.cpp
                        OpenSim/Extension/Tools/MeasurementSet.cpp
                        OpenSim/Extension/Tools/ModelScaler.cpp
                        OpenSim/Extension/Tools/ScaleTool.cpp
                        )

set(MY_LIBRARY_SOURCES
    lib/IKRunner.cpp
    lib/ModelEditor.cpp
    lib/SingleTrial.cpp
    lib/Utils.cpp
)

set(MY_LIBRARY_HEADERS
    lib/IKRunner.h
    lib/ModelEditor.h
    lib/Participant.h
    lib/SingleTrial.h
    lib/Utils.h
)

set (Plugin_HEADERS     ${MY_LIBRARY_HEADERS}
                        ${Plugin_Simbody_HEADERS}
                        ${Plugin_OpenSim_Analyses_HEADERS}
                        ${Plugin_OpenSim_Common_HEADERS}
                        ${Plugin_OpenSim_Simulation_HEADERS}
                        ${Plugin_OpenSim_Tools_HEADERS})

set (Plugin_SOURCES     ${MY_LIBRARY_SOURCES}
                        ${Plugin_Simbody_SOURCES}
                        ${Plugin_OpenSim_Analyses_SOURCES}
                        ${Plugin_OpenSim_Common_SOURCES}
                        ${Plugin_OpenSim_Simulation_SOURCES}
                        ${Plugin_OpenSim_Tools_SOURCES}
)
# Build Shared Library
add_library(${PLUGIN_NAME} ${Plugin_HEADERS} ${Plugin_SOURCES} )
# Specify the include directory
target_include_directories(${PLUGIN_NAME}
    PUBLIC ${CMAKE_SOURCE_DIR}/Simbody/include   # For this target and any target that links to it
    PUBLIC ${CMAKE_SOURCE_DIR}/lib
)
target_link_libraries(${PLUGIN_NAME} ${OpenSim_LIBRARIES})
target_compile_definitions(${PLUGIN_NAME} PRIVATE OSIMPLUGIN_EXPORTS)

set_target_properties(
    ${PLUGIN_NAME} PROPERTIES
    DEFINE_SYMBOL OSIMPLUGIN_EXPORTS
    PROJECT_LABEL "Libraries - ${PLUGIN_NAME}")

# install_targets(/plugins RUNTIME_DIRECTORY /plugins  ${PLUGIN_NAME})

# Thread Pool Lib
# ----------------------------
if(MSVC)
    add_compile_options(/permissive- /Zc:__cplusplus)
endif()
set(CPM_DOWNLOAD_LOCATION ${CMAKE_BINARY_DIR}/CPM.cmake)
if(NOT(EXISTS ${CPM_DOWNLOAD_LOCATION}))
    file(DOWNLOAD https://github.com/cpm-cmake/CPM.cmake/releases/latest/download/CPM.cmake ${CPM_DOWNLOAD_LOCATION})
endif()
include(${CPM_DOWNLOAD_LOCATION})

CPMAddPackage("gh:bshoshany/thread-pool@5.0.0")
add_library(BS_thread_pool INTERFACE)
target_include_directories(BS_thread_pool INTERFACE ${${CPM_LAST_PACKAGE_NAME}_SOURCE_DIR}/include)


# Build Demo Main
# If necessary, use the RELATIVE flag, otherwise each source file may be listed
# with full pathname. RELATIVE may makes it easier to extract an executable name
# automatically.
# file( GLOB APP_SOURCES RELATIVE app/*.cxx )
file( GLOB APP_SOURCES app/*.cpp )
foreach( testsourcefile ${APP_SOURCES} )
    # Cut off the file extension and directory path
    get_filename_component( testname ${testsourcefile} NAME_WE )
    add_executable( ${testname} ${testsourcefile} )
    # Make sure YourLib is linked to each app
    target_link_libraries( ${testname} ${PLUGIN_NAME} BS_thread_pool )

endforeach( testsourcefile ${APP_SOURCES} )


# Specify the source and destination paths
set(SOURCE_DATA_FOLDER "${CMAKE_SOURCE_DIR}/data")
set(DESTINATION_BIN_FOLDER "${CMAKE_BINARY_DIR}/bin")

# Copy the contents of the data folder to the bin folder
file(GLOB DATA_FILES "${SOURCE_DATA_FOLDER}/*")  # Get all files in the data folder

# Copy each file individually
foreach(file ${DATA_FILES})
    file(COPY ${file} DESTINATION ${DESTINATION_BIN_FOLDER})
endforeach()