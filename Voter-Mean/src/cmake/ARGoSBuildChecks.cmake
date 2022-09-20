#
# Check for correct GCC version
# On Linux it must be >= 4.8.1
#
macro(CHECK_GCC MIN_VERSION)
  execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  if(GCC_VERSION VERSION_GREATER ${MIN_VERSION} OR GCC_VERSION VERSION_EQUAL ${MIN_VERSION})
    message(STATUS "GCC/G++ version >= ${MIN_VERSION}")
  else(GCC_VERSION VERSION_GREATER ${MIN_VERSION} OR GCC_VERSION VERSION_EQUAL ${MIN_VERSION})
    message(FATAL_ERROR "You need to have at least GCC/G++ version ${MIN_VERSION}!")
  endif(GCC_VERSION VERSION_GREATER ${MIN_VERSION} OR GCC_VERSION VERSION_EQUAL ${MIN_VERSION})
endmacro(CHECK_GCC)

#
# Set variables depending on current compiler
#
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  # using Clang
  set(ARGOS_START_LIB_GROUP)
  set(ARGOS_END_LIB_GROUP)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  # using GCC
  set(ARGOS_START_LIB_GROUP -Wl,--start-group)
  set(ARGOS_END_LIB_GROUP -Wl,--end-group)

  if(APPLE)
    # On Apple it must be >= 4.8.1
    check_gcc(4.8.1)
  else(APPLE)
    # On Linux it must be >= 4.8.1
    check_gcc(4.8.1)
  endif(APPLE)
endif()

#
# Check if ARGoS is installed
#
find_package(ARGoS COMPONENTS entities prototype OPTIONAL_COMPONENTS qtopengl)
if(NOT ARGOS_FOUND)
  message(FATAL_ERROR "ARGoS was not found.")
endif(NOT ARGOS_FOUND)
include_directories(${ARGOS_INCLUDE_DIR})

#
# Check for PThreads
#
find_package(Pthreads)
if(NOT PTHREADS_FOUND)
 message(FATAL_ERROR "Required library pthreads not found.")
endif(NOT PTHREADS_FOUND)
add_definitions(${PTHREADS_DEFINITIONS})
include_directories(${PTHREADS_INCLUDE_DIR})

#
# Check for Lua 5.2
#
find_package(Lua52)
if(LUA52_FOUND)
  set(ARGOS_WITH_LUA ON)
  include_directories(${LUA_INCLUDE_DIR})
endif(LUA52_FOUND)

