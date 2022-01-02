CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# https://cmake.org/pipermail/cmake/2009-June/030311.html
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "MinSizeRel" CACHE STRING
      "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

set( PROJECT hobby-stm32scope)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set( MCU_VENDOR STM32)
#set( MCU_FAMILY F1)
set( MCU_FAMILY G4)
string(TOLOWER "${MCU_VENDOR}" MCU_VENDOR_LOWER_CASE)
string(TOLOWER "${MCU_FAMILY}" MCU_FAMILY_LOWER_CASE)
add_definitions( -D${MCU_VENDOR}${MCU_FAMILY})
add_definitions( -DNDEBUG)

set( STM32_FAMILY ${MCU_FAMILY})
set( CMAKE_TOOLCHAIN_FILE cmake/gcc_stm32.cmake)
if(${MCU_FAMILY} STREQUAL F1)
    set( STM32_CHIP STM32F103C8)
else()
    set( STM32_CHIP STM32G431KB)
endif()

include(gcc_stm32)

# libopencm3 build flags
set( FP_FLAGS "-mfloat-abi=softfp -mfpu=fpv4-sp-d16" )
set( EXTRA_C_FLAGS "-std=c99")

project(${PROJECT})

INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set( LIBRARIES
	libopencm3
	)

foreach( LIBRARY ${LIBRARIES})
    add_subdirectory (${LIBRARY} ${LIBRARY})
    include_directories( ${LIBRARY}/include)
endforeach()

include_directories(${LIBOPENCM3_INCLUDE_DIRS})

set( PROJECT_SOURCES
    src/main.c
    src/control.c
    )

set( PROJECT_HEADERS
    include/types.h
    src/adc.h
    src/bsp.h
    src/control.h
    )

if(${MCU_FAMILY} STREQUAL F1)
    set( PROJECT_SOURCES ${PROJECT_SOURCES}
        src/adc_f1.c
        src/bsp_f1.c
        )
else()
    set( PROJECT_SOURCES ${PROJECT_SOURCES}
        src/adc_g4.c
        src/bsp_g4.c
        )
endif()

set( CMAKE_EXE_LINKER_FLAGS ${LIBOPENCM3_LINKER_FLAGS} CACHE INTERNAL "executable linker flags")
set( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs --specs=nosys.specs")

if(${MCU_FAMILY} STREQUAL F1)
    SET(STM32_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/src/stm32-h103.ld)
else()
    SET(STM32_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/src/stm32g431kb.ld)
endif()

ADD_EXECUTABLE(${PROJECT} ${PROJECT_SOURCES} ${PROJECT_HEADERS} ${PROJECT_SCHEMES})

TARGET_LINK_LIBRARIES(${PROJECT} ${LIBRARIES})

STM32_SET_TARGET_PROPERTIES(${CMAKE_PROJECT_NAME})
STM32_ADD_HEX_BIN_TARGETS(${CMAKE_PROJECT_NAME})
STM32_PRINT_SIZE_OF_TARGETS(${CMAKE_PROJECT_NAME})

include(stm32_load_rules)
