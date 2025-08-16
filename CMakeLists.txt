cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

project(joltc C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_INSTALL_MESSAGE LAZY)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Determine if engine is built as a subproject (using add_subdirectory)
# or if it is the master project.
if (NOT DEFINED JPH_MASTER_PROJECT)
    set(JPH_MASTER_PROJECT OFF)
    if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
        set(JPH_MASTER_PROJECT ON)
        message(STATUS "CMake version: ${CMAKE_VERSION}")
    endif ()
endif ()

if (JPH_MASTER_PROJECT)
    # Configure CMake global variables
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif ()

if (DEFINED JPH_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE ${JPH_BUILD_TYPE})
endif ()

# Set the default CMAKE_BUILD_TYPE to Release.
# This should be done before the project command since the latter can set
# CMAKE_BUILD_TYPE itself (it does so for nmake).
if (JPH_MASTER_PROJECT AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Distribution" CACHE STRING "The default build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES})
endif ()

if (MSVC AND JPH_MASTER_PROJECT)
    if (NOT DEFINED CMAKE_MSVC_RUNTIME_LIBRARY)
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    endif ()
endif ()

include(GNUInstallDirs)
include(FetchContent)

# When turning this option on, the library will be compiled using assertions. By default asserts are enabled in Debug build.
option(USE_ASSERTS "Enable asserts" OFF)

# When turning this option on, the library will be compiled using doubles for positions. This allows for much bigger worlds.
option(DOUBLE_PRECISION "Use double precision math" OFF)

if (CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    # When turning this option on, the library will be compiled with debug symbols
    option(GENERATE_DEBUG_SYMBOLS "Generate debug symbols" ON)

    # When turning this option on, the library will override the default CMAKE_CXX_FLAGS_DEBUG/RELEASE values, otherwise they will use the platform defaults
    option(OVERRIDE_CXX_FLAGS "Override CMAKE_CXX_FLAGS_DEBUG/RELEASE" OFF)
else ()
    # When turning this option on, the library will be compiled with debug symbols
    option(GENERATE_DEBUG_SYMBOLS "Generate debug symbols" OFF)

    # When turning this option on, the library will override the default CMAKE_CXX_FLAGS_DEBUG/RELEASE values, otherwise they will use the platform defaults
    option(OVERRIDE_CXX_FLAGS "Override CMAKE_CXX_FLAGS_DEBUG/RELEASE" ON)
endif ()

# When turning this option on, the library will be compiled in such a way to attempt to keep the simulation deterministic across platforms
option(CROSS_PLATFORM_DETERMINISTIC "Cross platform deterministic" OFF)

# When turning this option on, the library will be compiled for ARM (aarch64-linux-gnu), requires compiling with clang
option(CROSS_COMPILE_ARM "Cross compile to aarch64-linux-gnu" OFF)

# When turning this on, in Debug and Release mode, the library will emit extra code to ensure that the 4th component of a 3-vector is kept the same as the 3rd component
# and will enable floating point exceptions during simulation to detect divisions by zero.
# Note that this currently only works using MSVC. Clang turns Float2 into a SIMD vector sometimes causing floating point exceptions (the option is ignored).
option(FLOATING_POINT_EXCEPTIONS_ENABLED "Enable floating point exceptions" ON)

# When turning this on, the library will be compiled with C++ exceptions enabled.
# This adds some overhead and Jolt doesn't use exceptions so by default it is off.
option(CPP_EXCEPTIONS_ENABLED "Enable C++ exceptions" OFF)

# When turning this on, the library will be compiled with C++ RTTI enabled.
# This adds some overhead and Jolt doesn't use RTTI so by default it is off.
option(CPP_RTTI_ENABLED "Enable C++ RTTI" OFF)

# Use 32-bit object layers to support more bits in ObjectLayerPairFilterMask
set(OBJECT_LAYER_BITS 32)

# Select X86 processor features to use (if everything is off it will be SSE2 compatible)
option(USE_SSE4_1 "Enable SSE4.1" ON)
option(USE_SSE4_2 "Enable SSE4.2" ON)
option(USE_AVX "Enable AVX" ON)
option(USE_AVX2 "Enable AVX2" ON)
option(USE_AVX512 "Enable AVX512" OFF)
option(USE_LZCNT "Enable LZCNT" ON)
option(USE_TZCNT "Enable TZCNT" ON)
option(USE_F16C "Enable F16C" ON)
option(USE_FMADD "Enable FMADD" ON)

# Enable SIMD for the WASM build. Note that this is currently off by default since not all browsers support this.
# See: https://caniuse.com/?search=WebAssembly%20SIMD (Safari got support in March 2023 and was the last major browser to get support).
option(USE_WASM_SIMD "Enable SIMD for WASM" OFF)

# Enable all warnings
option(ENABLE_ALL_WARNINGS "Enable all warnings and warnings as errors" OFF)

# Setting to periodically trace broadphase stats to help determine if the broadphase layer configuration is optimal
option(TRACK_BROADPHASE_STATS "Track Broadphase Stats" OFF)

# Setting to periodically trace narrowphase stats to help determine which collision queries could be optimized
option(TRACK_NARROWPHASE_STATS "Track Narrowphase Stats" OFF)

# Enable the debug renderer in the Debug and Release builds. Note that DEBUG_RENDERER_IN_DISTRIBUTION will override this setting.
option(DEBUG_RENDERER_IN_DEBUG_AND_RELEASE "Enable debug renderer in Debug and Release builds" OFF)

# Setting to enable the debug renderer in all builds.
# Note that enabling this reduces the performance of the library even if you're not drawing anything.
option(DEBUG_RENDERER_IN_DISTRIBUTION "Enable debug renderer in all builds" OFF)

# Enable the profiler in Debug and Release builds. Note that PROFILER_IN_DISTRIBUTION will override this setting.
option(PROFILER_IN_DEBUG_AND_RELEASE "Enable the profiler in Debug and Release builds" OFF)

# Enable the profiler in all builds.
# Note that enabling this reduces the performance of the library.
option(PROFILER_IN_DISTRIBUTION "Enable the profiler in all builds" OFF)

# Setting this option will force the library to use malloc/free instead of allowing the user to override the memory allocator
option(DISABLE_CUSTOM_ALLOCATOR "Disable support for a custom memory allocator" OFF)

# Setting this option will force the library to use the STL vector instead of the custom Array class
option(USE_STD_VECTOR "Use std::vector instead of own Array class" OFF)

# Setting this option will compile the ObjectStream class and RTTI attribute information
option(ENABLE_OBJECT_STREAM "Compile the ObjectStream class and RTTI attribute information" OFF)

option(JPH_SAMPLES "Build samples" ${JPH_MASTER_PROJECT})

# Define standard configurations (Debug, Release, Distribution)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release;Distribution")

# Search for local JoltPhysics first
if (JPH_MASTER_PROJECT AND EXISTS "${CMAKE_SOURCE_DIR}/../JoltPhysics/Build/CMakeLists.txt")
    message(STATUS "Using local JoltPhysics from ${CMAKE_SOURCE_DIR}/../JoltPhysics")

    FetchContent_Declare(
            JoltPhysics
            SOURCE_DIR "${CMAKE_SOURCE_DIR}/../JoltPhysics"
            SOURCE_SUBDIR "Build"
    )
else ()
    message(STATUS "Local Jolt not found, download from remote")

    FetchContent_Declare(
            JoltPhysics
            GIT_REPOSITORY "https://github.com/jrouwe/JoltPhysics"
            GIT_TAG v5.3.0
            SOURCE_SUBDIR "Build"
    )
endif ()
FetchContent_MakeAvailable(JoltPhysics)

if (XCODE)
    # Ensure that we enable SSE4.2 for the x86_64 build, XCode builds multiple architectures
    set_property(TARGET Jolt PROPERTY XCODE_ATTRIBUTE_OTHER_CPLUSPLUSFLAGS[arch=x86_64] "$(inherited) -msse4.2 -mpopcnt")
endif ()

if (MSVC)
    # Fill in the path to the asan libraries
    set(CLANG_LIB_PATH "\"$(VSInstallDir)\\VC\\Tools\\Llvm\\x64\\lib\\clang\\${CMAKE_CXX_COMPILER_VERSION}\\lib\\windows\"")

    # 64 bit architecture
    set(CMAKE_VS_PLATFORM_TOOLSET_HOST_ARCHITECTURE "x64")

    # Set general compiler flags
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:__cplusplus /Gm- /MP /nologo /diagnostics:classic /FC /fp:except- /Zc:inline")

    # Enable warnings
    if (ENABLE_ALL_WARNINGS)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Wall /WX")
    endif ()

    # Set compiler flag for disabling RTTI
    if (NOT CPP_RTTI_ENABLED)
        # Set compiler flag for disabling RTTI
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GR-")
    else ()
        # Set compiler flag for enabling RTTI
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GR")
    endif ()

    if (NOT CPP_EXCEPTIONS_ENABLED)
        # Remove any existing compiler flag that enables exceptions
        string(REPLACE "/EHsc" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

        # Disable warning about STL and compiler-generated types using noexcept when exceptions are disabled
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4577")
    else ()
        # Enable exceptions
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
    endif ()

    set(CMAKE_CXX_FLAGS_DISTRIBUTION "${CMAKE_CXX_FLAGS_RELEASE}")

    # Set linker flags
    set(CMAKE_EXE_LINKER_FLAGS "/SUBSYSTEM:WINDOWS /ignore:4221")
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
        if (CROSS_PLATFORM_DETERMINISTIC)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:precise")
        else ()
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fp:fast") # Clang doesn't use fast math because it cannot be turned off inside a single compilation unit
        endif ()
    elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /showFilenames")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Qunused-arguments") # Clang emits warnings about unused arguments such as /MP and /GL
    endif ()
else ()
    # Enable warnings
    if (ENABLE_ALL_WARNINGS)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")
    endif ()

    # Set compiler flag for disabling RTTI
    if (NOT CPP_RTTI_ENABLED)
        # Set compiler flag for disabling RTTI
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti")
    else ()
        # Set compiler flag for enabling RTTI
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frtti")
    endif ()

    # Disable exception-handling
    if (NOT CPP_EXCEPTIONS_ENABLED)
        # Set compiler flag for disabling exception-handling
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions")
    else ()
        # Set compiler flag for enabling exception-handling
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions")
    endif ()

    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        # Also disable -Wstringop-overflow or it will generate false positives that can't be disabled from code when link-time optimizations are enabled
        # Also turn off automatic fused multiply add contractions, there doesn't seem to be a way to do this selectively through the macro JPH_PRECISE_MATH_OFF
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-stringop-overflow -ffp-contract=off")
    else ()
        # Do not use -ffast-math since it cannot be turned off in a single compilation unit under clang, see Core.h
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffp-model=precise")

        # On clang 14 and later we can turn off float contraction through a pragma, older versions and deterministic versions need it off always, see Core.h
        if (CMAKE_CXX_COMPILER_VERSION LESS 14 OR CROSS_PLATFORM_DETERMINISTIC)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffp-contract=off")
        endif ()

        # Cross compiler flags
        if (CROSS_COMPILE_ARM)
            set(CMAKE_CXX_FLAGS "--target=aarch64-linux-gnu ${CMAKE_CXX_FLAGS}")
        endif ()
    endif ()

    # See https://github.com/jrouwe/JoltPhysics/issues/922. When compiling with DOUBLE_PRECISION=YES and CMAKE_OSX_DEPLOYMENT_TARGET=10.12 clang triggers a warning that we silence here.
    if ("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin" AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -faligned-allocation")
    endif ()

    # Set compiler flags for various configurations
    if (OVERRIDE_CXX_FLAGS)
        set(CMAKE_CXX_FLAGS_DEBUG "-O3")
        set(CMAKE_CXX_FLAGS_RELEASE "-O3")
    endif ()
    set(CMAKE_CXX_FLAGS_DISTRIBUTION "${CMAKE_CXX_FLAGS_RELEASE}")

    # Set linker flags
    if (NOT ("${CMAKE_SYSTEM_NAME}" STREQUAL "Windows"))
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")
    endif ()
endif ()

# Set linker flags
set(CMAKE_EXE_LINKER_FLAGS_DISTRIBUTION "${CMAKE_EXE_LINKER_FLAGS_RELEASE}")
set(CMAKE_SHARED_LINKER_FLAGS_DISTRIBUTION "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}")

SET_INTERPROCEDURAL_OPTIMIZATION()

set(SOURCE_FILES
        include/joltc/constants.h
        include/joltc/enums.h
        include/joltc/joltc.h
        include/joltc/types.h
        include/joltc/Math/Mat44.h
        include/joltc/Math/Quat.h
        include/joltc/Math/RMat44.h
        include/joltc/Math/RVec3.h
        include/joltc/Math/Transform.h
        include/joltc/Math/Vector3.h
        include/joltc/Physics/Body/Body.h
        include/joltc/Physics/Body/BodyInterface.h
        src/include/Math/Mat44.hpp
        src/include/Math/Quat.hpp
        src/include/Math/RMat44.hpp
        src/include/Math/RVec3.hpp
        src/include/Math/Transform.hpp
        src/include/Math/Vector3.hpp
        src/include/Physics/Body/BodyInterface.hpp
        src/Math/Vector3.cpp
        src/Math/RVec3.cpp
        src/Math/Quat.cpp
        src/Math/Mat44.cpp
        src/joltc.cpp
        src/joltc_assert.cpp
        src/Math/RMat44.cpp
        src/include/Physics/Body/Body.hpp
        src/Physics/Body/Body.cpp
        src/Physics/Body/BodyInterface.cpp
        src/include/Physics/Collision/CollisionGroup.hpp
        include/joltc/Physics/Collision/CollisionGroup.h
        include/joltc/Physics/Collision/GroupFilter.h
        src/include/Physics/Collision/GroupFilter.hpp
        src/Physics/Collision/GroupFilter.cpp
        src/Physics/Collision/CollisionGroup.cpp
        src/Physics/Body/BodyCreationSettings.cpp
        src/include/Physics/Body/BodyCreationSettings.hpp
        include/joltc/Physics/Body/BodyCreationSettings.h
        include/joltc/Physics/Collision/Shape/Shape.h
        src/include/Physics/Collision/Shape/Shape.hpp
        src/Physics/Collision/Shape/Shape.cpp
        include/joltc/Geometry/AABox.h
        src/include/Geometry/AABox.hpp
        include/joltc/Physics/Body/MassProperties.h
        src/include/Physics/Body/MassProperties.hpp
        src/Physics/Body/MassProperties.cpp
        src/include/Physics/Collision/PhysicsMaterial.hpp
        include/joltc/Physics/Collision/PhysicsMaterial.h
        src/Physics/Collision/PhysicsMaterial.cpp
        include/joltc/Physics/Collision/ShapeFilter.h
        src/include/Physics/Collision/ShapeFilter.hpp
        src/Physics/Collision/ShapeFilter.cpp
        src/include/Physics/Collision/RayCast.hpp
        include/joltc/Physics/Collision/RayCast.h
        include/joltc/Physics/SoftBody/SoftBodyCreationSettings.h
        src/include/Physics/SoftBody/SoftBodyCreationSettings.hpp
        src/Physics/SoftBody/SoftBodyCreationSettings.cpp
        include/joltc/Physics/Body/MotionProperties.h
        src/include/Physics/Body/MotionProperties.hpp
        src/Physics/Body/MotionProperties.cpp
)

add_library(joltc STATIC ${SOURCE_FILES})

set(JPH_CAPI_VALUE "extern __attribute__((visibility(\"default\")))")
target_compile_definitions(joltc PUBLIC JPH_CAPI=${JPH_CAPI_VALUE} JPH_API_CALL=$<$<BOOL:${WIN32}>:__cdecl>)

target_include_directories(joltc PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(joltc PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/include>)
target_link_libraries(joltc PRIVATE Jolt)

if (MSVC)
    # Debug information
    target_compile_options(joltc PRIVATE $<$<CONFIG:Debug>:/Zi>)

    # Enable full optimization in dev/release
    target_compile_options(joltc PRIVATE $<$<CONFIG:Debug>:/Od> $<$<NOT:$<CONFIG:Debug>>:/Ox>)

    # Inline function expansion
    target_compile_options(joltc PRIVATE /Ob2)

    # Enable intrinsic functions in dev/release
    target_compile_options(joltc PRIVATE $<$<NOT:$<CONFIG:Debug>>:/Oi>)

    # Favor fast code
    target_compile_options(joltc PRIVATE /Ot)

    # Enable fiber-safe optimizations in dev/release
    target_compile_options(joltc PRIVATE $<$<NOT:$<CONFIG:Debug>>:/GT>)

    # Enable string pooling
    target_compile_options(joltc PRIVATE /GF)

    # Use security checks only in debug
    target_compile_options(joltc PRIVATE $<$<CONFIG:DEBUG>:/sdl> $<$<NOT:$<CONFIG:DEBUG>>:/sdl->)
else ()

endif ()

install(TARGETS joltc
        EXPORT joltc
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/joltc.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/Jolt
)

if (JPH_SAMPLES)
    add_subdirectory(samples)
endif ()

if (CMAKE_GENERATOR_PLATFORM)
    message(STATUS "CMAKE_GENERATOR_PLATFORM: ${CMAKE_GENERATOR_PLATFORM}")
endif ()

message(STATUS "CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if (CMAKE_OSX_ARCHITECTURES)
    message(STATUS "CMAKE_OSX_ARCHITECTURES: ${CMAKE_OSX_ARCHITECTURES}")
endif ()
