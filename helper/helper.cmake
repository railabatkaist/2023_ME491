set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/Eigen3)

find_package(Eigen3 REQUIRED)
include_directories (${EIGEN3_INCLUDE_DIRS})
message(${EIGEN3_INCLUDE_DIRS})

function(create_executable app_name file_name)
    add_executable(${app_name} ${file_name})

    if(APPLE)
        execute_process(COMMAND sysctl -q hw.optional.arm64
                OUTPUT_VARIABLE _sysctl_stdout
                ERROR_VARIABLE _sysctl_stderr
                RESULT_VARIABLE _sysctl_result
                )
        if(_sysctl_result EQUAL 0 AND _sysctl_stdout MATCHES "hw.optional.arm64: 1")
            target_compile_options(${app_name} PRIVATE -mcpu=apple-m1)
        endif()
    endif()

    target_include_directories(${app_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_compile_definitions(${app_name} PRIVATE RESOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/resource)
endfunction()