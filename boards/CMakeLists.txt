# Board configuration. Each supported board has a pin-only header,
# boards/board_<name>.h, and a settings file, boards/sdkconfig.<name>. The
# Kconfig board choice picks one; everything else includes "board.h" and
# never a board_<name>.h directly (tools/check_layering.sh enforces this).
idf_component_register(INCLUDE_DIRS ".")

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    set(header "board_${CONFIG_DUMP3411_BOARD_NAME}.h")
    if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/${header}")
        message(FATAL_ERROR "Board '${CONFIG_DUMP3411_BOARD_NAME}' has no boards/${header}")
    endif()
    target_compile_definitions(${COMPONENT_LIB} INTERFACE DUMP3411_BOARD_HEADER="${header}")
endif()
