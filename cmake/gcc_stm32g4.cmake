SET(CMAKE_C_FLAGS "-mthumb -fno-builtin -mcpu=cortex-m4 -Wall -std=gnu99 -ffunction-sections -fdata-sections -fomit-frame-pointer -mabi=aapcs -fno-unroll-loops -ffast-math -ftree-vectorize" CACHE INTERNAL "c compiler flags")
SET(CMAKE_CXX_FLAGS "-mthumb -fno-builtin -mcpu=cortex-m4 -Wall -std=c++14 -ffunction-sections -fdata-sections -fomit-frame-pointer -mabi=aapcs -fno-unroll-loops -ffast-math -ftree-vectorize" CACHE INTERNAL "cxx compiler flags")
SET(CMAKE_ASM_FLAGS "-mthumb -mcpu=cortex-m4 -x assembler-with-cpp" CACHE INTERNAL "asm compiler flags")

SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections -mthumb -mcpu=cortex-m4 -mabi=aapcs" CACHE INTERNAL "executable linker flags")
SET(CMAKE_MODULE_LINKER_FLAGS "-mthumb -mcpu=cortex-m4 -mabi=aapcs" CACHE INTERNAL "module linker flags")
SET(CMAKE_SHARED_LINKER_FLAGS "-mthumb -mcpu=cortex-m4 -mabi=aapcs" CACHE INTERNAL "shared linker flags")

SET(STM32_CHIP_TYPES 431xx 441xx 473xx 483xx 474xx 484xx 491xx 4A1xx CACHE INTERNAL "stm32g4 chip types")
SET(STM32_CODES "431.." "441.." "473.." "483.." "474.." "484.." "491.." "4A1..")

MACRO(STM32_GET_CHIP_TYPE CHIP CHIP_TYPE)
    STRING(REGEX REPLACE "^[sS][tT][mM]32[gG](4[34789A][134].[468BCDEFG]).*$" "\\1" STM32_CODE ${CHIP})
    SET(INDEX 0)
    FOREACH(C_TYPE ${STM32_CHIP_TYPES})
        LIST(GET STM32_CODES ${INDEX} CHIP_TYPE_REGEXP)
        IF(STM32_CODE MATCHES ${CHIP_TYPE_REGEXP})
            SET(RESULT_TYPE ${C_TYPE})
        ENDIF()
        MATH(EXPR INDEX "${INDEX}+1")
    ENDFOREACH()
    SET(${CHIP_TYPE} ${RESULT_TYPE})
ENDMACRO()

MACRO(STM32_GET_CHIP_PARAMETERS CHIP FLASH_SIZE RAM_SIZE CCRAM_SIZE)
message("-----> ${CHIP}")
    STRING(REGEX REPLACE "^[sS][tT][mM]32[gG](4[34789A][134].[68BCDEFG]).*" "\\1" STM32_CODE ${CHIP})
    STRING(REGEX REPLACE "^[sS][tT][mM]32[gG]4[34789A][134].([68BCDEFG]).*" "\\1" STM32_SIZE_CODE ${CHIP})

    IF(STM32_SIZE_CODE STREQUAL "6")
        SET(FLASH "32K")
    ELSEIF(STM32_SIZE_CODE STREQUAL "8")
        SET(FLASH "64K")
    ELSEIF(STM32_SIZE_CODE STREQUAL "B")
        SET(FLASH "128K")
    ELSEIF(STM32_SIZE_CODE STREQUAL "C")
        SET(FLASH "256K")
    ELSEIF(STM32_SIZE_CODE STREQUAL "D")
        SET(FLASH "384K")
    ELSEIF(STM32_SIZE_CODE STREQUAL "E")
        SET(FLASH "512K")
    ELSEIF(STM32_SIZE_CODE STREQUAL "F")
        SET(FLASH "768K")
    ELSEIF(STM32_SIZE_CODE STREQUAL "G")
        SET(FLASH "1024K")
    ENDIF()

    STM32_GET_CHIP_TYPE(${CHIP} TYPE)

    IF(${TYPE} STREQUAL 431xx)
        SET(RAM "32K")
    ELSEIF(${TYPE} STREQUAL 441xx)
        SET(RAM "32K")
    ELSEIF(${TYPE} STREQUAL 491xx)
        SET(RAM "112K")
    ELSEIF(${TYPE} STREQUAL 4A1xx)
        SET(RAM "112K")
    ELSEIF(${TYPE} STREQUAL 473xx)
        SET(RAM "128K")
    ELSEIF(${TYPE} STREQUAL 474xx)
        SET(RAM "128K")
    ELSEIF(${TYPE} STREQUAL 483xx)
        SET(RAM "128K")
    ELSEIF(${TYPE} STREQUAL 484xx)
        SET(RAM "128K")
    ENDIF()

    SET(${FLASH_SIZE} ${FLASH})
    SET(${RAM_SIZE} ${RAM})
    SET(${CCRAM_SIZE} "0K")
ENDMACRO()

FUNCTION(STM32_SET_CHIP_DEFINITIONS TARGET CHIP_TYPE)
    LIST(FIND STM32_CHIP_TYPES ${CHIP_TYPE} TYPE_INDEX)
    IF(TYPE_INDEX EQUAL -1)
        MESSAGE(FATAL_ERROR "Invalid/unsupported STM32G4 chip type: ${CHIP_TYPE}")
    ENDIF()
    GET_TARGET_PROPERTY(TARGET_DEFS ${TARGET} COMPILE_DEFINITIONS)
    IF(TARGET_DEFS)
        SET(TARGET_DEFS "STM32G4;STM32G${CHIP_TYPE};${TARGET_DEFS}")
    ELSE()
        SET(TARGET_DEFS "STM32G4;STM32G${CHIP_TYPE}")
    ENDIF()
    SET_TARGET_PROPERTIES(${TARGET} PROPERTIES COMPILE_DEFINITIONS "${TARGET_DEFS}")
ENDFUNCTION()
