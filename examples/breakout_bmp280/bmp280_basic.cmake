set(OUTPUT_NAME bmp280_basic_demo)

add_executable(
  ${OUTPUT_NAME}
  bmp280_basic.cpp
)

# Pull in pico libraries that we need
target_link_libraries(${OUTPUT_NAME} pico_stdlib bmp280)

    # enable usb and uart output
    pico_enable_stdio_usb(${OUTPUT_NAME} 1)
    pico_enable_stdio_uart(${OUTPUT_NAME} 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(${OUTPUT_NAME})
