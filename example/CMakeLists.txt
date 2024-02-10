if (TARGET tinyusb_device)
    add_executable(hello_usb
            hello_usb.c
            logflash.c
            encoders.c
            map.c
            VL53L1X_api.c
            VL53L1X_calibration.c
            VL53L1X_platform.c
            )

    # pull in common dependencies
    target_link_libraries(hello_usb 
            pico_stdlib 
            pico_flash 
            hardware_gpio 
            hardware_pwm 
            hardware_i2c  
            hardware_adc 
            pico_cyw43_arch_none 
            pico_multicore 
            hardware_flash)

    target_include_directories(hello_usb PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)

    # Pull in the API
    # include(../library/import.cmake)

    # enable usb output, disable uart output
    pico_enable_stdio_usb(hello_usb 1)
    pico_enable_stdio_uart(hello_usb 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(hello_usb)

    add_executable(flash_test
            logflash.c
            flash_test.c
            map.c
            )

    # pull in common dependencies
    target_link_libraries(flash_test 
            pico_stdlib   
            pico_multicore 
            hardware_flash
            pico_cyw43_arch_none 
            pico_flash 
            hardware_rtc)

    target_include_directories(flash_test PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
    pico_enable_stdio_usb(flash_test 1)
    pico_enable_stdio_uart(flash_test 0)
    pico_add_extra_outputs(flash_test)

    add_executable(flash_erase
            logflash.c
            flash_erase.c
            )

    # pull in common dependencies
    target_link_libraries(flash_erase 
            pico_stdlib    
            hardware_flash 
            pico_flash 
            pico_cyw43_arch_none)

    target_include_directories(flash_erase PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
    pico_enable_stdio_usb(flash_erase 1)
    pico_enable_stdio_uart(flash_erase 0)
    pico_add_extra_outputs(flash_erase)

    add_executable(flash_read_all
            logflash.c
            flash_read_all.c
            )

    # pull in common dependencies
    target_link_libraries(flash_read_all 
            pico_stdlib    
            hardware_flash
            pico_flash 
            pico_cyw43_arch_none)

    target_include_directories(flash_read_all PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
    pico_enable_stdio_usb(flash_read_all 1)
    pico_enable_stdio_uart(flash_read_all 0)
    pico_add_extra_outputs(flash_read_all)

    add_executable(test_odometry
            encoders.c
            test_odometry.c
            )

    # pull in common dependencies
    target_link_libraries(test_odometry 
            pico_stdlib)

    target_include_directories(test_odometry PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
    pico_enable_stdio_usb(test_odometry 1)
    pico_enable_stdio_uart(test_odometry 0)
    pico_add_extra_outputs(test_odometry)
elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()