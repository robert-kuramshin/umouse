if (TARGET tinyusb_device)
    add_executable(hello_usb
            hello_usb.c
            encoders.c
            map.c
            VL53L1X_api.c
            VL53L1X_calibration.c
            VL53L1X_platform.c
            engine.c
            )

    # pull in common dependencies
    target_link_libraries(hello_usb  pico_stdlib
                            hardware_gpio hardware_pwm hardware_i2c hardware_adc pico_cyw43_arch_none pico_multicore)

    target_include_directories(hello_usb PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)

    # Pull in the API
    # include(../library/import.cmake)

    # enable usb output, disable uart output
    pico_enable_stdio_usb(hello_usb 1)
    pico_enable_stdio_uart(hello_usb 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(hello_usb)

    # add url via pico_set_program_url
    # example_auto_set_url(hello_usb)
elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()