if (TARGET tinyusb_device)
        add_executable(I2C_receiver
                I2C_receiver.c
                )

        # Pull in basic dependencies
        target_link_libraries(I2C_receiver i2c_slave pico_stdlib hardware_i2c)

        # enable USB output, disable UART output
        pico_enable_stdio_usb(I2C_receiver 1)
        pico_enable_stdio_uart(I2C_receiver 0)

        # create map/bin/hex file etc.
        pico_add_extra_outputs(I2C_receiver)

elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()



