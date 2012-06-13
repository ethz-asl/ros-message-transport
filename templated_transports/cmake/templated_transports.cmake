


#GENERATE_PLUGIN_DESCRIPTION(SRC_FILE, DEST_FILE)
MACRO (GENERATE_PLUGIN_DESCRIPTION)
    # Set PLUGIN_SRC and PLUGIN_OUTPUT)
    PARSE_ARGUMENTS(PLUGIN "SRC;OUTPUT" "" ${ARGN})
    set_source_files_properties(${PLUGIN_SRC} PROPERTIES GENERATED TRUE)
    rosbuild_find_ros_package(templated_transports)

    execute_process(
        COMMAND m4 "${templated_transports_PACKAGE_PATH}/default_plugins_template.m4" "${PLUGIN_SRC}" 
       OUTPUT_FILE ${PLUGIN_OUTPUT}
       WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )

ENDMACRO (GENERATE_PLUGIN_DESCRIPTION)


