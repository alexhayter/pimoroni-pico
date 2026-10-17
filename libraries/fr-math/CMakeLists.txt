add_library(fr_math 
    ${CMAKE_CURRENT_LIST_DIR}/FR_math.c
)

target_include_directories(fr_math INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(fr_math pico_stdlib)