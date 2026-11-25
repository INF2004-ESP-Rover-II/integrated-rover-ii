# Checks apply to project targets, not Pico SDK or sensor-driver libraries.
include_guard(GLOBAL)

function(barr_c_target target_name)
    get_target_property(project_sources ${target_name} SOURCES)
    foreach(source_file IN LISTS project_sources)
        if(source_file MATCHES "^(src/|camera_monitor/|\\.\\./esp32_s3_vision/main/).*\\.c$")
            set_property(SOURCE ${source_file} APPEND PROPERTY COMPILE_OPTIONS
                -Wall -Wextra -Wformat=2 -Wstrict-prototypes
                -Wmissing-prototypes -Wswitch-default
                -Werror=implicit-function-declaration -Werror=return-type
                -Werror=parentheses -Werror=uninitialized
            )
        endif()
    endforeach()
endfunction()

find_package(Python3 QUIET COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND AND NOT TARGET barr_c_check)
    add_custom_target(barr_c_check
        COMMAND "${Python3_EXECUTABLE}"
                "${CMAKE_CURRENT_LIST_DIR}/../tools/check_barr_c.py" --format
        WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/.."
        COMMENT "Checking project-owned firmware layout and BARR-C mechanical rules"
        VERBATIM
    )
endif()
