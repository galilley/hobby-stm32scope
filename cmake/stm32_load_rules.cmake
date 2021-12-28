cmake_minimum_required( VERSION 2.8)

##Programmer options
add_custom_target(ocd
    COMMAND openocd
    ARGS -f ${PROJECT_SOURCE_DIR}/cmake/stm32f1_stlinkv2.cfg -c "program ${PROJECT} verify reset exit"
    DEPENDS ${PROJECT}
    COMMENT "Load firmware to device...")

add_custom_target(dfu
    COMMAND sudo dfu-util
    ARGS -a 0 -s 0x08000000:leave -D ${PROJECT}.bin -R
    DEPENDS ${PROJECT}.bin
    COMMENT "Load firmware to device...")

add_custom_target(dfu_no_sudo
    COMMAND dfu-util
    ARGS -a 0 -s 0x08000000:leave -D ${PROJECT}.bin -R
    DEPENDS ${PROJECT}.bin
    COMMENT "Load firmware to device...")

add_custom_target(dfubin
    COMMAND sudo dfu-util
    ARGS -a 0 -s 0x08000000:leave -D ${PROJECT_SOURCE_DIR}/bin/${PROJECT}.bin -R
    #DEPENDS ${PROJECT}.bin
    COMMENT "Load previosly saved firmware to device...")
